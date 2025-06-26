// HIL.cpp – Hardware-in‑the‑Loop controller (Kalman + Quintic trajectory)
// ---------------------------------------------------------------------------
//  • Reads spherical‑coordinate sensor frames over a serial port.
//  • Uses a 6‑state constant‑velocity Kalman filter (x,y,z + v) instead of the
//    crude low‑pass.  RLS hooks are provided (compile‑time switch) for future
//    parameter/bias estimation.
//  • Tracks an *external* quintic‑polynomial trajectory (C^2‑continuous) rather
//    than simply hovering at the measured pose.  The quintic coefficients can
//    be regenerated on‑the‑fly (e.g. every reaching task) but here we demo a
//    single segment from the robot’s measured start pose to a hard‑coded goal.
//  • Feeds signed step frequencies → Arduino UNO R3 → EM556S stepper drivers.

//  Run:
//      ./hil /dev/ttyUSB0 /dev/ttyUSB1          # sensorPort  arduinoPort
//
// ---------------------------------------------------------------------------

#define ASIO_STANDALONE
#include <asio.hpp>
#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <array>
#include <cmath>
#include <iomanip>
#include <deque>
#include <mutex>
#include <cstring>  // std::memcpy
#include "SocketUtils.hpp"  // send_all()
#include <filesystem>
#include <regex>
#include <optional>

#include "RobotModel.hpp"
#include "Controller.hpp"
#include "HardcodedParams.hpp"
#include "HIL.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Constants (match SIL, but tweakable at runtime)
// ---------------------------------------------------------------------------
constexpr double kCtrlHz      = 100.0;     // Control loop frequency [Hz]
constexpr double kCtrlDt      = 1.0 / kCtrlHz;
constexpr unsigned kSensorBaud  = 115200U;
constexpr unsigned kArduinoBaud = 250000U;

//---------------------------------------------------------------------------
// Helper: emit one 128-byte Frame (wire-identical to the SIL stream)
//---------------------------------------------------------------------------
static inline void send_frame(socket_t sock,
                              double t,
                              const Eigen::Vector3d& x,
                              const Eigen::Vector3d& x_dot,
                              const Eigen::Vector3d& theta,
                              const Eigen::Vector3d& theta_dot,
                              const Eigen::Vector3d& tau)
{
    Frame f;
    f.t = t;
    std::memcpy(f.x,         x.data(),         3*sizeof(double));
    std::memcpy(f.x_dot,     x_dot.data(),     3*sizeof(double));
    std::memcpy(f.theta,     theta.data(),     3*sizeof(double));
    std::memcpy(f.theta_dot, theta_dot.data(), 3*sizeof(double));
    std::memcpy(f.tau,       tau.data(),       3*sizeof(double));
    send_all(sock, reinterpret_cast<const char*>(&f), sizeof(f));
}

// ---------------------------------------------------------------------------
// Serial helper (minimal ASIO wrapper)
// ---------------------------------------------------------------------------
class SerialPort {
public:
    SerialPort(asio::io_context& io, const std::string& path, unsigned baud)
        : port_(io) {
        asio::error_code ec;  port_.open(path, ec);
        if (ec) throw std::runtime_error("Cannot open " + path + ": " + ec.message());
        port_.set_option(asio::serial_port_base::baud_rate(baud));
        port_.set_option(asio::serial_port_base::character_size(8));
        port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    }
    void write(const uint8_t* d, std::size_t n){ asio::write(port_, asio::buffer(d,n)); }
    std::string readLine() { asio::streambuf sbuf; asio::read_until(port_,sbuf,'\n');
        std::istream is(&sbuf); std::string l; std::getline(is,l); return l; }
private:
    asio::serial_port port_;
};

// ---------------------------------------------------------------------------
// Sensor frame: "t,r,theta,phi" (spherical)
// ---------------------------------------------------------------------------
struct SensorFrame { double t=0,r=0,th=0,phi=0; };
inline bool parseSensorLine(const std::string& s, SensorFrame& o){
    return std::sscanf(s.c_str(), "%lf,%lf,%lf,%lf",&o.t,&o.r,&o.th,&o.phi)==4; }

inline Eigen::Vector3d sph2cart(double r,double th,double ph,const Eigen::Vector3d& c){
    return {c.x()+r*std::sin(th)*std::cos(ph),
            c.y()+r*std::sin(th)*std::sin(ph),
            c.z()+r*std::cos(th)}; }

// ---------------------------------------------------------------------------
// 6‑state constant‑velocity Kalman filter  (x,y,z,vx,vy,vz)
// ---------------------------------------------------------------------------
class KalmanPosVel {
public:
    KalmanPosVel(double dt){ setDt(dt); }
    void setDt(double dt){ dt_=dt; A_.setIdentity(); for(int i=0;i<3;++i) A_(i,i+3)=dt; Q_= Eigen::Matrix<double,6,6>::Identity()*1e-5; }
    void setR(double var){ R_=Eigen::Matrix3d::Identity()*var; }
    void init(const Eigen::Vector3d& x0){ x_.head<3>()=x0; x_.tail<3>().setZero(); P_.setIdentity(); init_=true; }
    void predict(){ if(!init_) return; x_=A_*x_; P_=A_*P_*A_.transpose()+Q_; }
    void update(const Eigen::Vector3d& z){ if(!init_) { init(z); return; }
        Eigen::Vector3d y = z - H_*x_; Eigen::Matrix<double,3,3> S = H_*P_*H_.transpose()+R_;
        Eigen::Matrix<double,6,3> K = P_*H_.transpose()*S.inverse(); x_+=K*y; P_=(I_-K*H_)*P_; }
    Eigen::Vector3d pos() const { return x_.head<3>(); }
    Eigen::Vector3d vel() const { return x_.tail<3>(); }
private:
    double dt_ = kCtrlDt; bool init_=false;
    Eigen::Matrix<double,6,6> A_{Eigen::Matrix<double,6,6>::Identity()}, P_{Eigen::Matrix<double,6,6>::Identity()}, Q_;
    const Eigen::Matrix<double,6,6> I_{Eigen::Matrix<double,6,6>::Identity()};
    const Eigen::Matrix<double,3,6> H_{ (Eigen::Matrix<double,3,6>()<<Eigen::Matrix3d::Identity(),Eigen::Matrix3d::Zero()).finished() };
    Eigen::Matrix3d R_{Eigen::Matrix3d::Identity()*1e-4};
    Eigen::Matrix<double,6,1> x_{Eigen::Matrix<double,6,1>::Zero()};
};

// Optional PC→HIL trajectory streaming port (3rd CLI arg)
constexpr unsigned kHostBaud  = 460800U;

// ---------------------------------------------------------------------------
// Trajectory streaming helpers
//    • expected line format:  "P,t,x,y,z,xd,yd,zd,xdd,ydd,zdd"
// ---------------------------------------------------------------------------

class TrajectoryRingBuffer {
public:
    void push(const TrajectoryPoint& tp){
        std::lock_guard<std::mutex> lk(m_);
        // keep strictly increasing time
        if(!q_.empty() && tp.t<=q_.back().t) return;
        q_.push_back(tp);
        if(q_.size()>kMax) q_.pop_front();
    }
    bool interpolate(double t, TrajectoryPoint& out) const {
        std::lock_guard<std::mutex> lk(m_);
        if(q_.size()<2 || t<q_.front().t || t>q_.back().t) return false;
        auto it = std::lower_bound(q_.begin(),q_.end(),t,[](const TrajectoryPoint& a,double tt){return a.t<tt;});
        if(it==q_.begin()) return false; // shouldn't happen
        const TrajectoryPoint& p1=*it; const TrajectoryPoint& p0=*std::prev(it);
        double s=(t-p0.t)/(p1.t-p0.t);
        out.t = t;
        out.x = (1-s)*p0.x + s*p1.x;
        out.x_dot = (1-s)*p0.x_dot + s*p1.x_dot;
        out.x_ddot = (1-s)*p0.x_ddot + s*p1.x_ddot;
        return true;
    }
private:
    std::deque<TrajectoryPoint> q_;
    mutable std::mutex m_;
    static constexpr size_t kMax = 2000;
};

// ---------------------------------------------------------------------------
// Driver packet helper  "F,fx,fy,fz,dirBits\n"
// ---------------------------------------------------------------------------
inline void sendFrequencies(SerialPort& ard,const Eigen::Vector3d& f){
    uint8_t dir=0; Eigen::Vector3d fp=f; for(int i=0;i<3;++i){ if(fp[i]<0){dir|=(1u<<i); fp[i]=-fp[i];} }
    std::ostringstream oss; oss<<'F'<<','<<std::fixed<<std::setprecision(1)<<fp[0]<<','<<fp[1]<<','<<fp[2]<<','<<int(dir)<<'\n';
    const std::string& msg=oss.str(); ard.write(reinterpret_cast<const uint8_t*>(msg.data()),msg.size()); }

// Arduino finder
static std::optional<std::string> find_arduino_uno()
{
    namespace fs = std::filesystem;

    // ① Fast path: udev already makes cute symlinks here
    const fs::path byIdDir{"/dev/serial/by-id"};
    std::regex unoPat{R"(.*(Arduino|UNO|CH340).*)", std::regex::icase};
    if (fs::exists(byIdDir))
        for (const auto& e : fs::directory_iterator(byIdDir))
            if (std::regex_match(e.path().string(), unoPat))
                return fs::canonical(e.path()).string();   // real /dev/ttyACM0…

    // ② Fallback: brute-force ttyACM* + ttyUSB* and ask udev for VID/PID
    //     ⇒ requires <libudev-dev> and -ludev when you link.
    #ifdef __linux__
    #include <libudev.h>
    udev* u = udev_new();
    if (!u) return {};
    udev_enumerate* en = udev_enumerate_new(u);
    udev_enumerate_add_match_subsystem(en, "tty");
    udev_enumerate_scan_devices(en);
    udev_list_entry* devs = udev_enumerate_get_list_entry(en);
    for (auto* ent = devs; ent; ent = udev_list_entry_get_next(ent))
    {
        const char* syspath = udev_list_entry_get_name(ent);
        udev_device* dev = udev_device_new_from_syspath(u, syspath);
        udev_device* usb = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if (!usb) { udev_device_unref(dev); continue; }
        const char* vid = udev_device_get_sysattr_value(usb, "idVendor");
        const char* pid = udev_device_get_sysattr_value(usb, "idProduct");
        if (!vid || !pid) { udev_device_unref(dev); continue; }

        // Known VID/PID pairs (original & clones)
        struct { const char* vid; const char* pid; } table[] = {
            {"2341","0043"},   // Arduino UNO R3
            {"2a03","0043"},   // Arduino.org UNO
            {"1a86","7523"},   // CH340/CH341
        };
        for (auto& v : table)
            if (!strcasecmp(vid,v.vid) && !strcasecmp(pid,v.pid))
                { auto node = udev_device_get_devnode(dev); std::string out(node); udev_device_unref(dev); udev_unref(u); return out; }

        udev_device_unref(dev);
    }
    udev_unref(u);
    #endif
    return {};        // none found
}


//------------------------------------------------------------------
//                            MAIN
//------------------------------------------------------------------
void run_hil_streaming(const std::vector<Waypoint>& trajectory,
                       socket_t                     sock,
                       const Eigen::Vector3d&       elbow_pos,
                       double                       l_arm_proth,
                       const std::string&           sensor_dev,
                       const std::string&           arduino_dev)
{
    asio::io_context io;
    try{
        SerialPort sensor  (io, sensor_dev , kSensorBaud );
        std::string ard_port = arduino_dev;
        if (ard_port == "auto") {
            if (auto p = find_arduino_uno()) {
                ard_port = *p;
                std::cerr << "[HIL] auto-detected Arduino on " << ard_port << '\n';
            } else {
                throw std::runtime_error(
                    "Cannot auto-detect Arduino UNO (try an explicit /dev/ttyXXX)");
            }
        }
        SerialPort arduino(io, ard_port, kArduinoBaud);

        // Push the pre-computed trajectory into the ring-buffer
        TrajectoryRingBuffer trajBuf;
        for(const Waypoint& wp : trajectory){
            TrajectoryPoint tp;
            tp.t  = wp.t;
            tp.x  = {wp.x[0]     , wp.x[1]     , wp.x[2]};
            tp.x_dot  = {wp.x_dot[0] , wp.x_dot[1] , wp.x_dot[2]};
            tp.x_ddot  = {wp.x_ddot[0], wp.x_ddot[1], wp.x_ddot[2]};
            trajBuf.push(tp);
        }

        // Robot + controller init ------------------------------------------------
        Eigen::Vector3d sphereCenter = elbow_pos;
        RobotDynamics robot; robot.loadHardcodedParams();
        robot.setElbowArm(sphereCenter, l_arm_proth);
        Controller ctrl;
        Eigen::Vector3d integralErr=Eigen::Vector3d::Zero();
        Eigen::Vector3d deltaPrev  =Eigen::Vector3d::Zero();
        Eigen::Vector3d lastTheta  (-2.76,-0.38,2.84);  // some plausible start
        Eigen::Vector3d prevTheta  = lastTheta;

        KalmanPosVel kf(kCtrlDt); kf.setR(1e-4);
        bool kfInited=false;

        double t0_global=-1;

        // Main RT loop -----------------------------------------------------------
        for(;;){
            const std::string line = sensor.readLine();
            SensorFrame sf; if(!parseSensorLine(line,sf)){ std::cerr<<"Bad sensor line: "<<line<<std::endl; continue; }
            if(t0_global<0) t0_global=sf.t;   // zero‑time at first packet
            const double tNow = sf.t - t0_global;

            // ── Online radial-bias estimation (same Kalman loop as SIL) ──
            static double r_bias = 0.0;          // m
            static double P      = 1e-3;         // variance
            constexpr double sigma_p = 3e-4;     // sensor noise  (m)
            constexpr double sigma_r = 1e-4;     // process noise (m·√s)

            const double r_meas = sf.r;                  // raw laser range ρ
            const double y      = r_meas - l_arm_proth;  // ρ − L  (identical to SIL)
            const double err    = y - r_bias;            // innovation

            const double K = P / (P + sigma_p * sigma_p);// Kalman gain
            r_bias += K * err;                           // state update
            P       = (1.0 - K) * P + sigma_r * sigma_r; // covariance time-update

            // Use the corrected range to build the Cartesian measurement
            const double r_est = l_arm_proth + r_bias;   // L + bias  → matches SIL
            const Eigen::Vector3d meas =
                sph2cart(r_est, sf.th, sf.phi, sphereCenter);

            // Kalman filtering ---------------------------------------------------
            kf.predict(); kf.update(meas);
            const Eigen::Vector3d x     = kf.pos();
            const Eigen::Vector3d x_dot = kf.vel();

            // Desired state ------------------------------------------------------
            Eigen::Vector3d x_des=x, x_desDot=Eigen::Vector3d::Zero(), x_desDDot=Eigen::Vector3d::Zero();
            TrajectoryPoint tpDes;
            if(trajBuf.interpolate(tNow,tpDes)){
                x_des     = tpDes.x;
                x_desDot  = tpDes.x_dot;
                x_desDDot = tpDes.x_ddot;
            }

            // Inverse kinematics -------------------------------------------------
            auto ik = robot.invKineSinglePoint(x_des,lastTheta);
            if(ik.valid) lastTheta = ik.theta; else std::cerr<<"IK failed keeping previous theta\n";
            Eigen::Vector3d thetaDot = (lastTheta - prevTheta)/kCtrlDt; prevTheta = lastTheta;

            // Controller ---------------------------------------------------------
            TrajectoryPoint des{tNow,x_des,x_desDot,x_desDDot};
            Eigen::Vector3d f_pulse = ctrl.computeMPCStepRateTorque(des,x,x_dot,x_desDDot,lastTheta,thetaDot,robot,kCtrlDt,integralErr,deltaPrev);

            // Stream to Arduino --------------------------------------------------
            sendFrequencies(arduino, f_pulse);

            // Mirror robot state back to the host (keeps Python UI unblocked)
            send_frame(sock, tNow,
                       x, x_dot,
                       lastTheta, thetaDot,
                       f_pulse);
        }

        #ifdef _WIN32
            shutdown(sock, SD_SEND);
        #else
            shutdown(sock, SHUT_WR);
        #endif

        }catch(const std::exception& ex){
            std::cerr << "[HIL] " << ex.what() << std::endl;
        }

}
