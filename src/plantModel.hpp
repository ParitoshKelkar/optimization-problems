#include <Eigen>
#include <Vector>

const double g = 9.8056;

class TireProps
{
    public:
        double mu_x_ash_f,mu_y_ash_f,Bx_ash_f,By_ash_f;
        double Cx_ash_f, Cy_ash_f, Ex_ash_f, Ey_ash_f;
        double mu_x_ash_r,mu_y_ash_r,Bx_ash_r,By_ash_r;
        double Cx_ash_r,Cy_ash_r,Ex_ash_r,Ey_ash_r; 

        TireProps(int surface = 1)
        {
            switch (surface)
            {
                case 1:
                default:
                    // Ashphalt front
                    this->mu_x_ash_f= 1.20 ; 
                    this->mu_y_ash_f= 0.935; 
                    this->Bx_ash_f= 11.7; 
                    this->By_ash_f= 8.86; 
                    this->Cx_ash_f= 1.69; 
                    this->Cy_ash_f= 1.19; 
                    this->Ex_ash_f= 0.377; 
                    this->Ey_ash_f= -1.21; 

                    // Ashpalt rear
                    this->mu_x_ash_r= 1.20 ; 
                    this->mu_y_ash_r= 0.961; 
                    this->Bx_ash_r= 11.1; 
                    this->By_ash_r= 9.30; 
                    this->Cx_ash_r= 1.69; 
                    this->Cy_ash_r= 1.19; 
                    this->Ex_ash_r= 0.362; 
                    this->Ey_ash_r= -1.11;   
                     
                case 2:
                    // Wet Ashphalt front
                    this->mu_x_ash_f= 1.06 ; 
                    this->mu_y_ash_f= 0.885; 
                    this->Bx_ash_f= 12; 
                    this->By_ash_f= 10.7; 
                    this->Cx_ash_f= 1.80; 
                    this->Cy_ash_f= 1.07; 
                    this->Ex_ash_f= 0.313; 
                    this->Ey_ash_f= -2.14; 

                    // Wet Ashpalt rear
                    this->mu_x_ash_r= 1.07 ; 
                    this->mu_y_ash_r= 0.911; 
                    this->Bx_ash_r= 11.5; 
                    this->By_ash_r= 11.3; 
                    this->Cx_ash_r= 1.80; 
                    this->Cy_ash_r= 1.07; 
                    this->Ex_ash_r= 0.300; 
                    this->Ey_ash_r= -1.97;  
                    
                case 3:
                    // Snow front
                    this->mu_x_ash_f= 0.407 ; 
                    this->mu_y_ash_f= 0.383; 
                    this->Bx_ash_f= 10.2; 
                    this->By_ash_f= 19.1; 
                    this->Cx_ash_f= 1.96; 
                    this->Cy_ash_f= 0.550; 
                    this->Ex_ash_f= 0.651; 
                    this->Ey_ash_f= -2.1; 

                    // Snow rear
                    this->mu_x_ash_r= 0.409; 
                    this->mu_y_ash_r= 0.394; 
                    this->Bx_ash_r= 9.71; 
                    this->By_ash_r= 20.0; 
                    this->Cx_ash_r= 1.96; 
                    this->Cy_ash_r= 0.550; 
                    this->Ex_ash_r= 0.624; 
                    this->Ey_ash_r= -1.93;  
            };
        };

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
    TireProps tire;

    VehicleParams(int surface):tire(surface){};
};

class Plant
{
    private:
        VehicleParams params; 
    public:
        Plant(const VehicleParams& params) : params(params){};
    
        double simulateForward(double steering_ang, std::vector<double> torques, std::vector<double> feedback)
        {
            double tau_f = torques[0];
            double tau_r = torques[1];

            // X,Y,psi,Vx,Vy,psiDot,theta,thetaDot,phi,phiDot,
            // omega_FL, omega_FR, omega_RR, omega_RL,
            // alpha_FL, alpha_FR, alpha_RR, alpha_RL
            double X= feedback[0], Y= feedback[1], psi= feedback[2];
            double Vx= feedback[3], Vy= feedback[4],psiDot = feedback[5];
            double theta = feedback[6], thetaDot= feedback[7];
            double phi = feedback[8], phiDot= feedback[9];
            double omega1=feedback[10],omega2=feedback[11],omega3=feedback[12],omega4=feedback[13];
            double alpha1=feedback[14],alpha2=feedback[15],alpha3=feedback[16],alpha4=feedback[17];
            
            // Calculate load transfer moments in pitch and roll
            double tau_theta= (params.K_theta*theta)+(params.D_theta*thetaDot);
            double tau_phi= (params.K_phi*phi)+(params.D_phi*phiDot);
            double tau_phif= (params.K_phif*phi)+(params.D_phif*phiDot);
            double tau_phir= (params.K_phir*phi)+(params.D_phir*phiDot);

            // Calculate Fz in front and rear axles
            double Fzf = (params.l_r*params.m*g + tau_theta)/(params.l);
            double Fzr = params.m*g - Fzf;

            // To be continued
        };
};
