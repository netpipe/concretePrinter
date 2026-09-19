// arm_sim.cpp
//
// 3-axis robotic arm G-code simulator
// OpenGL + GLEW + GLFW, no GLM.
//
// Defaults:
//   TCP port: 9999
//   Arm: 3R spherical arm
//        base yaw, shoulder pitch, elbow pitch
//   G-code XYZ moves the tool center point.
//
// Build examples are included after the code.

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #pragma comment(lib, "ws2_32.lib")
#else
  #include <arpa/inet.h>
  #include <cerrno>
  #include <fcntl.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <sys/types.h>
  #include <unistd.h>
#endif

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using std::string;
using std::vector;

// ---------------------------------------------------------------------------
// Small math helpers, no GLM.
// ---------------------------------------------------------------------------

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

static Vec3 operator+(const Vec3& a, const Vec3& b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 operator-(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 operator*(const Vec3& a, double s) {
    return Vec3{a.x * s, a.y * s, a.z * s};
}

static double length(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static string trim(const string& s) {
    size_t a = 0;
    size_t b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

static bool isNumericChar(char c) {
    return std::isdigit(static_cast<unsigned char>(c)) ||
           c == '+' || c == '-' || c == '.' ||
           c == 'e' || c == 'E';
}

// ---------------------------------------------------------------------------
// Socket abstraction.
// ---------------------------------------------------------------------------

#ifdef _WIN32
using socket_t = SOCKET;
static const socket_t INVALID_SOCK = INVALID_SOCKET;
#else
using socket_t = int;
static const socket_t INVALID_SOCK = -1;
#endif

static bool initNetwork() {
#ifdef _WIN32
    static WSADATA wsaData;
    return WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;
#else
    return true;
#endif
}

static void cleanupNetwork() {
#ifdef _WIN32
    WSACleanup();
#endif
}

static void closeSocket(socket_t s) {
    if (s == INVALID_SOCK) return;
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

static bool setNonBlocking(socket_t s) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(s, FIONBIO, &mode) == 0;
#else
    int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0) return false;
    return fcntl(s, F_SETFL, flags | O_NONBLOCK) >= 0;
#endif
}

static bool wouldBlock() {
#ifdef _WIN32
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

// ---------------------------------------------------------------------------
// Simulator state.
// ---------------------------------------------------------------------------

static double L1 = 120.0;          // Upper arm length, mm
static double L2 = 100.0;          // Forearm length, mm
static double shoulderZ = 40.0;    // Shoulder height above base, mm

static Vec3 HOME{150.0, 0.0, 40.0};
static Vec3 programmed = HOME;     // Programmed target position
static Vec3 current = HOME;        // Animated current position

static double feedMMPerMin = 3000.0;
static bool relativeMode = false;
static bool inchUnits = false;
static bool estop = false;

static int tcpPort = 9999;
static bool tcpEnabled = true;
static socket_t listenerSock = INVALID_SOCK;
static socket_t clientSock = INVALID_SOCK;
static string rxBuffer;

// Camera state.
static float camYaw = 45.0f;
static float camPitch = 25.0f;
static float camDist = 550.0f;
static bool mouseLeft = false;
static double lastMouseX = 0.0;
static double lastMouseY = 0.0;

static void workspaceLimits(double& rmin, double& rmax) {
    rmin = std::max(0.001, std::fabs(L1 - L2));
    rmax = std::max(rmin + 0.001, L1 + L2 - 0.001);
}

static Vec3 clampWorkspace(const Vec3& p) {
    double rmin, rmax;
    workspaceLimits(rmin, rmax);

    Vec3 d{p.x, p.y, p.z - shoulderZ};
    double r = length(d);

    if (r < 1e-9) {
        return Vec3{rmin, 0.0, shoulderZ};
    }

    if (r < rmin) {
        double s = rmin / r;
        d = d * s;
    } else if (r > rmax) {
        double s = rmax / r;
        d = d * s;
    }

    return Vec3{d.x, d.y, d.z + shoulderZ};
}

static void resetPositions() {
    double rmin, rmax;
    workspaceLimits(rmin, rmax);

    double hx = clampd((L1 + L2) * 0.70, rmin + 1.0, rmax - 1.0);
    HOME = Vec3{hx, 0.0, shoulderZ};
    programmed = HOME;
    current = HOME;
    estop = false;
}

// ---------------------------------------------------------------------------
// Network / G-code receiver.
// ---------------------------------------------------------------------------

static void sendAll(const string& s) {
    if (clientSock == INVALID_SOCK) return;

    size_t sent = 0;
    while (sent < s.size()) {
        int n = static_cast<int>(send(clientSock, s.data() + sent,
                                      static_cast<int>(s.size() - sent), 0));
        if (n <= 0) break;
        sent += static_cast<size_t>(n);
    }
}

static void sendStatus() {
    if (clientSock == INVALID_SOCK) return;

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "<Idle|MPos:%.3f,%.3f,%.3f|FS:%.1f,0>\r\n",
                  programmed.x, programmed.y, programmed.z,
                  feedMMPerMin);
    sendAll(string(buf));
}

static socket_t createListener(int port) {
    socket_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCK) return INVALID_SOCK;

    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<unsigned short>(port));

    if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        closeSocket(s);
        return INVALID_SOCK;
    }

    if (listen(s, 1) != 0) {
        closeSocket(s);
        return INVALID_SOCK;
    }

    if (!setNonBlocking(s)) {
        closeSocket(s);
        return INVALID_SOCK;
    }

    return s;
}

// Returns true if the sender should receive "ok".
static bool processLine(const string& raw) {
    string line = raw;

    // Remove comment.
    size_t comment = line.find(';');
    if (comment != string::npos) {
        line = line.substr(0, comment);
    }

    line = trim(line);
    if (line.empty()) return false;

    // Minimal GRBL-like real-time characters.
    if (line == "?") {
        sendStatus();
        return false;
    }
    if (line == "!" || line == "~") {
        // Feed hold / cycle start are accepted but ignored in this simulator.
        return false;
    }

    string u;
    u.reserve(line.size());
    for (char ch : line) {
        u.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
    }

    bool hasMove = false;
    bool didHome = false;
    bool didSetPosition = false;

    bool px = false, py = false, pz = false;
    double mx = 0.0, my = 0.0, mz = 0.0;

    size_t i = 0;
    while (i < u.size()) {
        unsigned char uc = static_cast<unsigned char>(u[i]);
        if (!std::isalpha(uc)) {
            ++i;
            continue;
        }

        char cmd = static_cast<char>(uc);
        size_t start = i;
        size_t j = i + 1;

        // Allow spaces between letter and number, e.g. "X 100".
        while (j < u.size() && std::isspace(static_cast<unsigned char>(u[j]))) {
            ++j;
        }

        size_t k = j;
        while (k < u.size() && isNumericChar(u[k])) {
            ++k;
        }

        double val = 0.0;
        if (k > j) {
            try {
                val = std::stod(u.substr(j, k - j));
            } catch (...) {
                val = 0.0;
            }
        }

        switch (cmd) {
            case 'G': {
                int g = static_cast<int>(val);

                if (g == 0 || g == 1) {
                    hasMove = true;
                } else if (g == 20) {
                    inchUnits = true;
                } else if (g == 21) {
                    inchUnits = false;
                } else if (g == 28) {
                    didHome = true;
                } else if (g == 90) {
                    relativeMode = false;
                } else if (g == 91) {
                    relativeMode = true;
                } else if (g == 92) {
                    didSetPosition = true;
                } else {
                    // Unknown G-codes are ignored.
                }
                break;
            }

            case 'M': {
                int m = static_cast<int>(val);

                if (m == 112) {
                    estop = true;
                    programmed = current;
                } else if (m == 999) {
                    estop = false;
                } else {
                    // Unknown M-codes are ignored.
                }
                break;
            }

            case 'X': {
                if (inchUnits) val *= 25.4;
                mx = val;
                px = true;
                break;
            }

            case 'Y': {
                if (inchUnits) val *= 25.4;
                my = val;
                py = true;
                break;
            }

            case 'Z': {
                if (inchUnits) val *= 25.4;
                mz = val;
                pz = true;
                break;
            }

            case 'F': {
                if (inchUnits) val *= 25.4;
                feedMMPerMin = std::max(1.0, val);
                break;
            }

            case 'E':
                // Extruder codes are ignored in this robot simulator.
                break;

            default:
                // Ignore other letters, including N, P, S, T, etc.
                break;
        }

        if (k > start) {
            i = k;
        } else {
            i = start + 1;
        }
    }

    if (px || py || pz) {
        hasMove = true;
    }

    if (didHome) {
        estop = false;
        programmed = clampWorkspace(HOME);
    } else if (didSetPosition) {
        Vec3 np = programmed;
        if (px) np.x = mx;
        if (py) np.y = my;
        if (pz) np.z = mz;
        programmed = clampWorkspace(np);
    } else if (hasMove && !estop) {
        Vec3 np = programmed;

        if (relativeMode) {
            if (px) np.x += mx;
            if (py) np.y += my;
            if (pz) np.z += mz;
        } else {
            if (px) np.x = mx;
            if (py) np.y = my;
            if (pz) np.z = mz;
        }

        programmed = clampWorkspace(np);
    }

    return true;
}

static void pollSockets() {
    if (!tcpEnabled || listenerSock == INVALID_SOCK) return;

    // Accept a new client if none is connected.
    if (clientSock == INVALID_SOCK) {
        socket_t s = accept(listenerSock, nullptr, nullptr);
        if (s != INVALID_SOCK) {
            clientSock = s;
            setNonBlocking(clientSock);
            rxBuffer.clear();

            // Looks enough like GRBL for many senders.
            sendAll("Grbl 1.1h ['$' for help]\r\n");
            sendAll("ok\r\n");

            std::cout << "G-code client connected.\n";
        } else if (!wouldBlock()) {
            std::cerr << "accept() failed.\n";
        }
    }

    // Receive data.
    if (clientSock != INVALID_SOCK) {
        char buf[4096];
        int n = static_cast<int>(recv(clientSock, buf, sizeof(buf), 0));

        if (n > 0) {
            rxBuffer.append(buf, static_cast<size_t>(n));

            // Guard against runaway buffers.
            if (rxBuffer.size() > 1024 * 1024) {
                rxBuffer.clear();
                closeSocket(clientSock);
                clientSock = INVALID_SOCK;
                std::cout << "G-code client disconnected: buffer overflow.\n";
                return;
            }

            // Handle real-time ? even if no newline yet.
            for (;;) {
                size_t qp = rxBuffer.find('?');
                if (qp == string::npos) break;
                rxBuffer.erase(qp, 1);
                sendStatus();
            }

            // Ignore ! and ~ if they appear as standalone bytes.
            for (;;) {
                size_t pp = rxBuffer.find_first_of("!~");
                if (pp == string::npos) break;
                rxBuffer.erase(pp, 1);
            }

            // Process complete lines.
            for (;;) {
                size_t pos = rxBuffer.find_first_of("\r\n");
                if (pos == string::npos) break;

                char eol = rxBuffer[pos];
                size_t eraseLen = 1;

                if (eol == '\r' && pos + 1 < rxBuffer.size() && rxBuffer[pos + 1] == '\n') {
                    eraseLen = 2;
                }

                string line = rxBuffer.substr(0, pos);
                rxBuffer.erase(0, pos + eraseLen);

                bool ack = processLine(line);
                if (ack) {
                    sendAll("ok\r\n");
                }
            }
        } else if (n == 0) {
            closeSocket(clientSock);
            clientSock = INVALID_SOCK;
            std::cout << "G-code client disconnected.\n";
        } else {
            if (!wouldBlock()) {
                closeSocket(clientSock);
                clientSock = INVALID_SOCK;
                std::cout << "G-code client disconnected by error.\n";
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Motion interpolation.
// ---------------------------------------------------------------------------

static void updateMotion(double dt) {
    if (estop) return;

    Vec3 d = programmed - current;
    double dist = length(d);

    double speedMMPerSec = feedMMPerMin / 60.0;
    double maxStep = speedMMPerSec * dt;

    if (dist <= maxStep || dist < 1e-9) {
        current = programmed;
    } else {
        current = current + d * (maxStep / dist);
    }
}

// ---------------------------------------------------------------------------
// 3R inverse kinematics.
//
// World convention:
//   Base at (0,0,0)
//   Shoulder at (0,0,shoulderZ)
//   +X is forward at zero base angle
//   +Z is up
//
// Joints:
//   q1: base rotation around Z
//   theta1: absolute upper-arm elevation in the vertical radial plane
//   theta2: absolute forearm elevation
//
// OpenGL drawing uses rotations around Y, where positive glRotatef(Y)
// pitches +X downward. Therefore drawing angles are negated elevations.
// ---------------------------------------------------------------------------

struct ArmAngles {
    double baseDeg = 0.0;
    double upperDeg = 0.0;
    double foreDeg = 0.0;
};

static ArmAngles solveIK(const Vec3& p) {
    ArmAngles a;

    double rmin, rmax;
    workspaceLimits(rmin, rmax);

    Vec3 d{p.x, p.y, p.z - shoulderZ};

    double rho = std::hypot(d.x, d.y);
    double z = d.z;
    double r = std::hypot(rho, z);

    r = clampd(r, rmin, rmax);

    double q1 = 0.0;
    if (rho > 1e-9) {
        q1 = std::atan2(d.y, d.x);
    }

    double alpha = std::atan2(z, rho);

    double c3 = (r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    c3 = clampd(c3, -1.0, 1.0);
    double q3 = std::acos(c3);

    double rr = std::max(r, 1e-9);
    double c1 = (L1 * L1 + rr * rr - L2 * L2) / (2.0 * L1 * rr);
    c1 = clampd(c1, -1.0, 1.0);
    double psi = std::acos(c1);

    // Elbow-up configuration.
    double theta1 = alpha + psi;

    a.baseDeg = q1 * 180.0 / M_PI;
    a.upperDeg = -theta1 * 180.0 / M_PI;

    // Forearm relative rotation for OpenGL:
    // absolute forearm elevation theta2 = theta1 - q3
    // relative elevation = theta2 - theta1 = -q3
    // OpenGL Y rotation = -relative elevation = +q3
    a.foreDeg = q3 * 180.0 / M_PI;

    return a;
}

// ---------------------------------------------------------------------------
// OpenGL drawing helpers.
// ---------------------------------------------------------------------------

static void drawAxes(float len) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    glColor3f(1.0f, 0.2f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(len, 0.0f, 0.0f);

    glColor3f(0.2f, 1.0f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, len, 0.0f);

    glColor3f(0.2f, 0.4f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, len);

    glEnd();
    glEnable(GL_LIGHTING);
}

static void drawGrid(float size, float step) {
    if (step <= 0.0f) return;

    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(0.32f, 0.34f, 0.36f);

    int n = static_cast<int>(size / step);
    for (int i = -n; i <= n; ++i) {
        float p = static_cast<float>(i) * step;

        glVertex3f(p, -size, 0.0f);
        glVertex3f(p, size, 0.0f);

        glVertex3f(-size, p, 0.0f);
        glVertex3f(size, p, 0.0f);
    }

    glEnd();

    // World axes.
    glBegin(GL_LINES);
    glColor3f(0.9f, 0.2f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(100.0f, 0.0f, 0.0f);

    glColor3f(0.2f, 0.9f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 100.0f, 0.0f);

    glColor3f(0.2f, 0.4f, 0.9f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 100.0f);
    glEnd();

    glEnable(GL_LIGHTING);
}

static void drawCross(const Vec3& p, float s) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 0.2f);

    glVertex3f(static_cast<float>(p.x - s), static_cast<float>(p.y), static_cast<float>(p.z));
    glVertex3f(static_cast<float>(p.x + s), static_cast<float>(p.y), static_cast<float>(p.z));

    glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y - s), static_cast<float>(p.z));
    glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y + s), static_cast<float>(p.z));

    glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z - s));
    glVertex3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z + s));

    glEnd();
    glEnable(GL_LIGHTING);
}

// Cylinder along local +X from x=0 to x=len.
static void drawCylinderX(float radius, float len, int slices = 20) {
    if (slices < 3) slices = 3;

    // Side.
    for (int i = 0; i < slices; ++i) {
        float a0 = static_cast<float>(2.0 * M_PI * i / slices);
        float a1 = static_cast<float>(2.0 * M_PI * (i + 1) / slices);

        float c0 = std::cos(a0);
        float s0 = std::sin(a0);
        float c1 = std::cos(a1);
        float s1 = std::sin(a1);

        glBegin(GL_QUADS);

        glNormal3f(0.0f, c0, s0);
        glVertex3f(0.0f, radius * c0, radius * s0);

        glNormal3f(0.0f, c1, s1);
        glVertex3f(0.0f, radius * c1, radius * s1);

        glNormal3f(0.0f, c1, s1);
        glVertex3f(len, radius * c1, radius * s1);

        glNormal3f(0.0f, c0, s0);
        glVertex3f(len, radius * c0, radius * s0);

        glEnd();
    }

    // Cap at x=0.
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (int i = 0; i <= slices; ++i) {
        float a = static_cast<float>(2.0 * M_PI * i / slices);
        glVertex3f(0.0f, radius * std::cos(a), radius * std::sin(a));
    }
    glEnd();

    // Cap at x=len.
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(len, 0.0f, 0.0f);
    for (int i = slices; i >= 0; --i) {
        float a = static_cast<float>(2.0 * M_PI * i / slices);
        glVertex3f(len, radius * std::cos(a), radius * std::sin(a));
    }
    glEnd();
}

// Cylinder along local +Z from z=0 to z=height.
static void drawCylinderZ(float radius, float height, int slices = 24) {
    if (slices < 3) slices = 3;

    // Side.
    for (int i = 0; i < slices; ++i) {
        float a0 = static_cast<float>(2.0 * M_PI * i / slices);
        float a1 = static_cast<float>(2.0 * M_PI * (i + 1) / slices);

        float c0 = std::cos(a0);
        float s0 = std::sin(a0);
        float c1 = std::cos(a1);
        float s1 = std::sin(a1);

        glBegin(GL_QUADS);

        glNormal3f(c0, s0, 0.0f);
        glVertex3f(radius * c0, radius * s0, 0.0f);

        glNormal3f(c1, s1, 0.0f);
        glVertex3f(radius * c1, radius * s1, 0.0f);

        glNormal3f(c1, s1, 0.0f);
        glVertex3f(radius * c1, radius * s1, height);

        glNormal3f(c0, s0, 0.0f);
        glVertex3f(radius * c0, radius * s0, height);

        glEnd();
    }

    // Bottom cap.
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (int i = slices; i >= 0; --i) {
        float a = static_cast<float>(2.0 * M_PI * i / slices);
        glVertex3f(radius * std::cos(a), radius * std::sin(a), 0.0f);
    }
    glEnd();

    // Top cap.
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, height);
    for (int i = 0; i <= slices; ++i) {
        float a = static_cast<float>(2.0 * M_PI * i / slices);
        glVertex3f(radius * std::cos(a), radius * std::sin(a), height);
    }
    glEnd();
}

static void drawArm() {
    ArmAngles a = solveIK(current);

    glEnable(GL_LIGHTING);

    // Base plate.
    glColor3f(0.25f, 0.25f, 0.28f);
    drawCylinderZ(38.0f, 6.0f, 28);

    // Base column.
    glColor3f(0.75f, 0.22f, 0.22f);
    drawCylinderZ(20.0f, static_cast<float>(shoulderZ), 28);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, static_cast<float>(shoulderZ));

    // Shoulder hub.
    glColor3f(0.85f, 0.65f, 0.20f);
    drawCylinderZ(13.0f, 8.0f, 22);

    // Joint 1: base yaw.
    glRotatef(static_cast<float>(a.baseDeg), 0.0f, 0.0f, 1.0f);

    // Joint 2: shoulder pitch.
    glRotatef(static_cast<float>(a.upperDeg), 0.0f, 1.0f, 0.0f);

    // Upper arm.
    glColor3f(0.20f, 0.55f, 0.90f);
    drawCylinderX(8.0f, static_cast<float>(L1), 20);

    // Move to elbow.
    glTranslatef(static_cast<float>(L1), 0.0f, 0.0f);

    // Joint 3: elbow pitch, relative to upper arm.
    glRotatef(static_cast<float>(a.foreDeg), 0.0f, 1.0f, 0.0f);

    // Forearm.
    glColor3f(0.25f, 0.80f, 0.45f);
    drawCylinderX(6.0f, static_cast<float>(L2), 20);

    // Tool center point.
    glTranslatef(static_cast<float>(L2), 0.0f, 0.0f);

    // Small tool holder centered on TCP.
    glColor3f(0.90f, 0.90f, 0.35f);
    glPushMatrix();
    glTranslatef(-8.0f, 0.0f, 0.0f);
    drawCylinderX(4.0f, 16.0f, 16);
    glPopMatrix();

    drawAxes(22.0f);

    glPopMatrix();
}

static void render(int width, int height) {
    if (width <= 0) width = 1;
    if (height <= 0) height = 1;

    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Projection.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float nearPlane = 1.0f;
    float farPlane = 20000.0f;
    float fovY = 45.0f * static_cast<float>(M_PI) / 180.0f;
    float top = nearPlane * std::tan(fovY * 0.5f);
    float aspect = static_cast<float>(width) / static_cast<float>(height);
    float right = top * aspect;

    glFrustum(-right, right, -top, top, nearPlane, farPlane);

    // Modelview / camera.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Headlight.
    GLfloat lightPos[] = {0.0f, 0.0f, 1.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    Vec3 viewTarget{0.0, 0.0, shoulderZ + 40.0};

    glTranslatef(0.0f, 0.0f, -camDist);
    glRotatef(camPitch, 1.0f, 0.0f, 0.0f);
    glRotatef(camYaw, 0.0f, 1.0f, 0.0f);
    glTranslatef(static_cast<float>(-viewTarget.x),
                 static_cast<float>(-viewTarget.y),
                 static_cast<float>(-viewTarget.z));

    drawGrid(400.0f, 25.0f);
    drawArm();
    drawCross(programmed, 12.0f);
}

// ---------------------------------------------------------------------------
// GLFW callbacks.
// ---------------------------------------------------------------------------

static void keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;

    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if (key == GLFW_KEY_R) {
        camYaw = 45.0f;
        camPitch = 25.0f;
        camDist = 550.0f;
    } else if (key == GLFW_KEY_H) {
        programmed = clampWorkspace(HOME);
    }
}

static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    (void)window;

    if (mouseLeft) {
        double dx = xpos - lastMouseX;
        double dy = ypos - lastMouseY;

        camYaw += static_cast<float>(dx * 0.30);
        camPitch += static_cast<float>(dy * 0.30);
        camPitch = static_cast<float>(clampd(camPitch, -89.0, 89.0));
    }

    lastMouseX = xpos;
    lastMouseY = ypos;
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        mouseLeft = (action == GLFW_PRESS);
        if (mouseLeft) {
            glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
        }
    }
}

static void scrollCallback(GLFWwindow* /*window*/, double /*xoffset*/, double yoffset) {
    camDist *= std::exp(static_cast<float>(-yoffset * 0.10));
    camDist = static_cast<float>(clampd(camDist, 50.0, 5000.0));
}

static void initGL() {
    glClearColor(0.08f, 0.09f, 0.10f, 1.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glShadeModel(GL_SMOOTH);
}

// ---------------------------------------------------------------------------
// Main.
// ---------------------------------------------------------------------------

static void printUsage(const char* exe) {
    std::cout <<
        "3-axis robotic arm G-code simulator\n"
        "\n"
        "Usage:\n"
        "  " << exe << " [options]\n"
        "\n"
        "Options:\n"
        "  --port N       TCP port to listen on, default 9999\n"
        "  --l1 MM        Upper arm length, default 120\n"
        "  --l2 MM        Forearm length, default 100\n"
        "  --height MM    Shoulder height, default 40\n"
        "  --no-tcp       Disable TCP receiver\n"
        "  --help         Show this help\n"
        "\n"
        "Controls:\n"
        "  Left mouse drag: orbit camera\n"
        "  Mouse wheel: zoom\n"
        "  R: reset camera\n"
        "  H: home simulator locally\n"
        "  Esc: quit\n";
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];

        auto needValue = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << opt << " requires a value.\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if (a == "--help" || a == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (a == "--port") {
            tcpPort = std::atoi(needValue("--port"));
        } else if (a == "--l1") {
            L1 = std::atof(needValue("--l1"));
        } else if (a == "--l2") {
            L2 = std::atof(needValue("--l2"));
        } else if (a == "--height") {
            shoulderZ = std::atof(needValue("--height"));
        } else if (a == "--no-tcp") {
            tcpEnabled = false;
        } else {
            std::cerr << "Unknown option: " << a << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    L1 = std::max(1.0, L1);
    L2 = std::max(1.0, L2);
    shoulderZ = std::max(0.0, shoulderZ);

    resetPositions();

    if (!initNetwork()) {
        std::cerr << "Network initialization failed.\n";
        return 1;
    }

    if (!glfwInit()) {
        std::cerr << "glfwInit() failed.\n";
        cleanupNetwork();
        return 1;
    }

    // Do not force a core profile. This simulator uses legacy OpenGL.
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(
        1280, 720,
        "3-Axis Robotic Arm G-code Simulator",
        nullptr, nullptr);

    if (!window) {
        std::cerr << "glfwCreateWindow() failed.\n";
        glfwTerminate();
        cleanupNetwork();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE;
    GLenum glewErr = glewInit();
    if (glewErr != GLEW_OK) {
        std::cerr << "GLEW error: " << glewGetErrorString(glewErr) << "\n";
    }

    // Clear possible errors generated by GLEW on some drivers.
    while (glGetError() != GL_NO_ERROR) {}

    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scrollCallback);

    initGL();

    if (tcpEnabled) {
        listenerSock = createListener(tcpPort);
        if (listenerSock == INVALID_SOCK) {
            std::cerr << "Failed to listen on TCP port " << tcpPort << ".\n";
            std::cerr << "Try another port, e.g. --port 12345\n";
            tcpEnabled = false;
        } else {
            std::cout << "Listening for G-code on tcp://0.0.0.0:" << tcpPort << "\n";
            std::cout << "Example test:\n";
            std::cout << "  nc 127.0.0.1 " << tcpPort << "\n";
            std::cout << "Then type:\n";
            std::cout << "  G21\n";
            std::cout << "  G90\n";
            std::cout << "  G1 X100 Y0 Z" << shoulderZ << " F3000\n";
        }
    }

    double prevTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        double now = glfwGetTime();
        double dt = now - prevTime;
        prevTime = now;

        dt = clampd(dt, 0.0, 0.1);

        pollSockets();
        updateMotion(dt);

        int w = 0, h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        render(w, h);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    closeSocket(clientSock);
    clientSock = INVALID_SOCK;

    closeSocket(listenerSock);
    listenerSock = INVALID_SOCK;

    glfwDestroyWindow(window);
    glfwTerminate();
    cleanupNetwork();

    return 0;
}