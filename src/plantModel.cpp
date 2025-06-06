#include "plantModel.hpp"

TireProps::TireProps(int surface)
{
    switch (surface)
            {
                case 1:
                default:
                    // Ashphalt front
                    this->mu_x_f= 1.20 ; 
                    this->mu_y_f= 0.935; 
                    this->Bx_f= 11.7; 
                    this->By_f= 8.86; 
                    this->Cx_f= 1.69; 
                    this->Cy_f= 1.19; 
                    this->Ex_f= 0.377; 
                    this->Ey_f= -1.21; 

                    // Ashpalt rear
                    this->mu_x_r= 1.20 ; 
                    this->mu_y_r= 0.961; 
                    this->Bx_r= 11.1; 
                    this->By_r= 9.30; 
                    this->Cx_r= 1.69; 
                    this->Cy_r= 1.19; 
                    this->Ex_r= 0.362; 
                    this->Ey_r= -1.11;   
                     
                case 2:
                    // Wet Ashphalt front
                    this->mu_x_f= 1.06 ; 
                    this->mu_y_f= 0.885; 
                    this->Bx_f= 12; 
                    this->By_f= 10.7; 
                    this->Cx_f= 1.80; 
                    this->Cy_f= 1.07; 
                    this->Ex_f= 0.313; 
                    this->Ey_f= -2.14; 

                    // Wet Ashpalt rear
                    this->mu_x_r= 1.07 ; 
                    this->mu_y_r= 0.911; 
                    this->Bx_r= 11.5; 
                    this->By_r= 11.3; 
                    this->Cx_r= 1.80; 
                    this->Cy_r= 1.07; 
                    this->Ex_r= 0.300; 
                    this->Ey_r= -1.97;  
                    
                case 3:
                    // Snow front
                    this->mu_x_f= 0.407 ; 
                    this->mu_y_f= 0.383; 
                    this->Bx_f= 10.2; 
                    this->By_f= 19.1; 
                    this->Cx_f= 1.96; 
                    this->Cy_f= 0.550; 
                    this->Ex_f= 0.651; 
                    this->Ey_f= -2.1; 

                    // Snow rear
                    this->mu_x_r= 0.409; 
                    this->mu_y_r= 0.394; 
                    this->Bx_r= 9.71; 
                    this->By_r= 20.0; 
                    this->Cx_r= 1.96; 
                    this->Cy_r= 0.550; 
                    this->Ex_r= 0.624; 
                    this->Ey_r= -1.93;  
            };
};

Eigen::Vector2d TireProps::pacjeka_tire_model(double slip_angle,double slip_ratio,double force_Z,bool tire_option, double epsilon)
{
    double mu_x, mu_y, Bx,By, Cx, Cy, Ex,Ey;

    if(!tire_option)
    {
         mu_x= this->mu_x_f, mu_y= this->mu_y_f;     
         Bx = this->Bx_f, By = this->By_f;
         Cx = this->Cx_f, Cy = this->Cy_f;
         Ex = this->Ex_f, Ey = this->Ey_f;
    }
    else
    {
         mu_x= this->mu_x_r, mu_y= this->mu_y_r;
         Bx = this->Bx_r, By = this->By_r;
         Cx = this->Cx_r, Cy = this->Cy_r;
         Ex = this->Ex_r, Ey = this->Ey_r;
    };
    Eigen::Vector2d F;
    
    // Longitudinal tire force
    F(0) = mu_x*force_Z*sin(Cx*atan(Bx*(1-Ex)*slip_ratio+Ex*std::atan(Bx*slip_ratio)));

    // Lateral tire force
    F(1)= mu_y*force_Z*sin(Cy*atan(By*(1-Ey)*slip_angle+Ey*atan(By*slip_angle)));

    // Tire curve coupling
    F(1) = F(1)*sqrt(1-((F(0)/(mu_x*force_Z))*(F(0)/(mu_x*force_Z)))+epsilon); 
};

Plant::Plant(const VehicleParams& params) : params(params){};

double Plant::sign_sigmoid(double V)
{
    return  -1+(2/(1+std::exp(-this->params.scale*V)));
};

double Plant::slip_angle(double Vx,double Vy)
{
    return  std::atan(this->sign_sigmoid(Vx)*Vy/std::sqrt(Vx*Vx+this->params.epsilon));
};

double Plant::slip_ratio(double Vx,double omega)
{
    return this->sign_sigmoid(Vx)*(this->params.Rw*omega-Vx)/(std::sqrt(Vx*Vx+this->params.epsilon));
};

double Plant::simulateForward(const double& steering_ang, const std::vector<double>& torques, const std::vector<double>& feedback)
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

            // Calculate Fz at individual tires
            double Fz1 = 1/2*(Fzf - 1/params.w*(tau_phif));
            double Fz2 = Fzf - Fz1;
            double Fz3 = 1/2*(Fzr - 1/params.w*(tau_phir));
            double Fz4 = Fzr - Fz3; 

            // Velocities at individual tires rotated by steering angles
            Eigen::Matrix2d rot1;
            rot1 << std::cos(steering_ang), -std::sin(steering_ang),
                    std::sin(steering_ang), std::cos(steering_ang);
            double Vx1_ur= Vx-(params.w*psiDot), Vx2_ur= Vx+(params.w*psiDot);
            double Vx3_ur= Vx1_ur, Vx4_ur= Vx2_ur;
            double Vy1_ur= Vy+(params.l_f*psiDot), Vy3_ur= Vy-(params.l_r*psiDot);
            double Vy2_ur= Vy1_ur, Vy4_ur= Vy3_ur;
            Eigen::Vector2d Vt1 = rot1*Eigen::Vector2d(Vy1_ur, Vx1_ur);
            double Vy1= Vt1(0), Vx1= Vt1(1);
            Eigen::Vector2d Vt2 = rot1*Eigen::Vector2d(Vy2_ur, Vx2_ur);
            double Vy2= Vt2(0), Vx2= Vt2(1);
            double Vy3= Vy3_ur, Vx3= Vx3_ur;
            double Vy4= Vy4_ur, Vx4= Vx4_ur;

            // Slip angles
            double alpha1_aux= this->slip_angle(Vx1,Vy1);
            double alpha2_aux= this->slip_angle(Vx2,Vy2);
            double alpha3_aux= this->slip_angle(Vx3,Vy3);
            double alpha4_aux= this->slip_angle(Vx4,Vy4);

            double alphadot1= -Vx1/(this->params.sig*(alpha1+alpha1_aux));
            double alphadot2= -Vx2/(this->params.sig*(alpha2+alpha2_aux));
            double alphadot3= -Vx3/(this->params.sig*(alpha3+alpha3_aux));
            double alphadot4= -Vx4/(this->params.sig*(alpha4+alpha4_aux));

            // Slip ratios
            double kappa1= this->slip_ratio(Vx1,omega1);
            double kappa2= this->slip_ratio(Vx2,omega2);
            double kappa3= this->slip_ratio(Vx3,omega3);
            double kappa4= this->slip_ratio(Vx4,omega4);

            // Tire forces
            Eigen::Vector2d F1 = this->params.tire->pacjeka_tire_model(alpha1,kappa1,Fz1,false,this->params.epsilon);
            Eigen::Vector2d F2 = this->params.tire->pacjeka_tire_model(alpha2,kappa2,Fz2,false,this->params.epsilon);
            Eigen::Vector2d F3 = this->params.tire->pacjeka_tire_model(alpha3,kappa3,Fz3,false,this->params.epsilon);
            Eigen::Vector2d F4 = this->params.tire->pacjeka_tire_model(alpha4,kappa4,Fz4,false,this->params.epsilon);

            // Need to rotate front tire forces with steering angle
            F1 = rot1*F1;
            F2 = rot1*F2;

            // Wheel Dynamics
            double omegadot1= (torques[0]/2-(this->params.Rw*F1(0)))/this->params.Iw;
            double omegadot2= (torques[0]/2-(this->params.Rw*F2(0)))/this->params.Iw;
            double omegadot3= (torques[1]/2-(this->params.Rw*F3(0)))/this->params.Iw;
            double omegadot4= (torques[1]/2-(this->params.Rw*F4(0)))/this->params.Iw;

            // Net Forces and Moments
            double Fx= F1(0)+F2(0)+F3(0)+F4(0);
            double Fy= F1(1)+F2(1)+F3(1)+F4(1);
            double Mz= this->params.l_f*(F1(1)+F2(1)) + this->params.w*(F2(0)-F1(0)) - this->params.l_r*(F3(1)+F4(1)) - this->params.w*(F4(1)+F3(1));

            //

        };