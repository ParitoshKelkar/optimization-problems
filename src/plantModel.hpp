#include <Eigen>
#include <Vector>
#include <cmath>
#include <memory>
#include <stdexcept>

const double g = 9.8056;

class TireProps
{
    public:
        double mu_x_f,mu_y_f,Bx_f,By_f;
        double Cx_f, Cy_f, Ex_f, Ey_f;
        double mu_x_r,mu_y_r,Bx_r,By_r;
        double Cx_r,Cy_r,Ex_r,Ey_r; 

        TireProps(int surface = 1);
        Eigen::Vector2d pacjeka_tire_model(double slip_angle,double slip_ratio,double force_Z,bool tire_option,double epsilon);
};

struct VehicleParams
{
    double m; // Mass in Kg
    double Ixx,Iyy,Izz; // Rotational inertia around X,Y and Z
    double l_f,l_r,l,w; //front to cg, cg to rear, wheelbase, trackwidth
    double Rw,Re,sig; // Wheel radius, effective radius and relaxation length
    double Iw; // Wheel inertia
    double h; // Height to roll center
    double K_theta, D_theta; // Pitch stiffness, pitch damping
    double K_phif, K_phir, K_phi, D_phif,D_phir, D_phi; // Roll stiffness and roll damping
    double epsilon, scale; // Integration integrity constants
    std::unique_ptr<TireProps> tire;
    VehicleParams(int surface) : tire(std::make_unique<TireProps>(surface)) {};
};

class Plant
{
    private:
        const VehicleParams& params; 
        double sign_sigmoid(double V) const;
        double slip_angle(double Vx,double Vy) const;
        double slip_ratio(double Vx,double omega) const;
    public:
        Plant(const VehicleParams& params);          
        std::vector<double> simulateForward(const double& steering_ang, const std::vector<double>& torques, const std::vector<double>& feedback) const;
};

class Simulation
{
    private:
        const double Ts = 0.01;
        const Plant& plant;
        std::vector<double> x0;
        double control_sampling_time;
        int N;
        std::vector<double> scalarMultiply(const std::vector<double>& vector, double scalar);
        std::vector<double> addVectors(const std::vector<double>& vec1, const std::vector<double>& vec2);
    public:
        Simulation(std::vector<double> x0,double control_sampling_time,Plant& plant);
        void propagateDynamics(const double& steering_ang, const std::vector<double>& torques,std::vector<double>& feedback);
};
