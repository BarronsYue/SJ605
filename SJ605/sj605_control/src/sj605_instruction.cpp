#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <stdio.h>
#include <math.h>
using namespace std;

#define PI 3.14159

// Matrix functions
vector<vector<double> > double_multi(vector<vector<double> > A, vector<vector<double> > B);
vector<vector<double> > inverse(double theta, double d, double a, double alpha);
vector<vector<double> > forward(double theta, double d, double a, double alpha);
void print_mat(vector<vector<double> > A);
vector<vector<double> > Rx(double theta);
vector<vector<double> > Ry(double theta);
vector<vector<double> > Rz(double theta);

// Callback functions for subscribing 
void callback3(control_msgs::JointControllerState msg)
{
    ROS_INFO("joint3 state = %f", msg.process_value);
}

void callback4(control_msgs::JointControllerState msg)
{
    ROS_INFO("joint4 state = %f", msg.process_value);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sj605");
    ros::NodeHandle n;
    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/sj605/joint1_position_controller/command", 10);
    ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/sj605/joint2_position_controller/command", 10);
    ros::Publisher joint3_pub = n.advertise<std_msgs::Float64>("/sj605/joint3_position_controller/command", 10);
    ros::Publisher joint4_pub = n.advertise<std_msgs::Float64>("/sj605/joint4_position_controller/command", 10);
    ros::Publisher joint5_pub = n.advertise<std_msgs::Float64>("/sj605/joint5_position_controller/command", 10);
    ros::Publisher joint6_pub = n.advertise<std_msgs::Float64>("/sj605/joint6_position_controller/command", 10);


    //ros::Subscriber joint3_sub = n.subscribe("/sj605/joint3_position_controller/state", 10, callback3);
    //ros::Subscriber joint4_sub = n.subscribe("/sj605/joint4_position_controller/state", 10, callback4);

    ros::Rate rate(1);

    std_msgs::Float64 joint1, joint2, joint3, joint4, joint5, joint6;

    double theta1, theta2, theta3, theta4, theta5, theta6;
    double px, py, pz;
    double Wx, Wy, Wz;
    double height, a2, d2, d4, d6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;
    double r11, r21, r31, r12, r22, r32, r13, r23, r33, qx, qy, qz;
    double rxy, rxyz, a, b, c, alpha, beta;
    char type;
    double rx, ry, rz;

    height = 0.73;                 // Height from the floor to the first joint
    a2 = 3.8;                    // D-H table parameters
    d2 = -0.081;
    d4 = 2.7;
    d6 = 1.25;
    alpha1 = PI/2;
    alpha2 = 0;
    alpha3 = PI/2;
    alpha4 = -PI/2;
    alpha5 = PI/2;
    alpha6 = 0;

    

    // Construct matrix
    vector<vector<double> > T, T_3_6, inv1, inv2, inv3, tem1, tem2;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row, temp_row_T_3_6, temp_row_inv1, temp_row_inv2, temp_row_inv3, temp_row_tem1, temp_row_tem2;

    vector<vector<double> > rotation;
    vector<double> temp_rotation;

    temp_row.resize(num_columns);
    temp_row_T_3_6.resize(num_columns);
    temp_row_inv1.resize(num_columns);
    temp_row_inv2.resize(num_columns);
    temp_row_inv3.resize(num_columns);
    temp_row_tem1.resize(num_columns);
    temp_row_tem2.resize(num_columns);
    temp_rotation.resize(num_columns);

    
    for (unsigned int i = 0; i < num_rows; i++)
    {
        T.push_back(temp_row);
        T_3_6.push_back(temp_row_T_3_6);
        inv1.push_back(temp_row_inv1);
        inv2.push_back(temp_row_inv2);
        inv3.push_back(temp_row_inv3);
        tem1.push_back(temp_row_tem1);
        tem2.push_back(temp_row_tem2);
        rotation.push_back(temp_rotation);

    }

   

    // Construct forward kinematics
    vector<vector<double> > A_0_3, A_0_5, A_0_6, A_0_1;
    vector<vector<double> > A_1_2, A_2_3, A_3_4, A_4_5, A_5_6;
    vector<vector<double> > mul1, mul2;
    vector<double> temp_row_0_3, temp_row_0_5, temp_row_0_6, temp_row_0_1;
    vector<double> temp_row_1_2, temp_row_2_3, temp_row_3_4, temp_row_4_5, temp_row_5_6;
    vector<double> temp_row_mul1, temp_row_mul2;

    temp_row_0_3.resize(num_columns);
    temp_row_0_5.resize(num_columns);
    temp_row_0_6.resize(num_columns);
    temp_row_0_1.resize(num_columns);
    temp_row_1_2.resize(num_columns);
    temp_row_2_3.resize(num_columns);
    temp_row_3_4.resize(num_columns);
    temp_row_4_5.resize(num_columns);
    temp_row_5_6.resize(num_columns);
    temp_row_mul1.resize(num_columns);
    temp_row_mul2.resize(num_columns);

    
    for (unsigned int i = 0; i < num_rows; i++)
    {
        A_0_3.push_back(temp_row_0_3);
        A_0_5.push_back(temp_row_0_5);
        A_0_6.push_back(temp_row_0_6);
        A_0_1.push_back(temp_row_0_1);
        A_1_2.push_back(temp_row_1_2);
        A_2_3.push_back(temp_row_2_3);
        A_3_4.push_back(temp_row_3_4);
        A_4_5.push_back(temp_row_4_5);
        A_5_6.push_back(temp_row_5_6);
        mul1.push_back(temp_row_mul1);
        mul2.push_back(temp_row_mul2);

    }


    joint1.data = 0;
    joint2.data = 0;
    joint3.data = 0;
    joint4.data = 0;
    joint5.data = 0;
    joint6.data = 0;


    while (ros::ok())
    {       
            cout << "*******************************************************************************" << endl;
            cout << "Choose your input type : " << endl << endl;
            cout << "f : forward, i : inverse" << endl << endl;
            cout << "Usage : " << endl;
            cout << endl;            
            cout << " f theta1 theta2 theta3 theta4 theta5 theta6" << endl;
            cout << endl;
            cout << "or" << endl << endl;
            cout << " i qx qy qz Rx Ry Rz" << endl << endl;
            cout << "Rx, Ry and Rz mean the angle of rotation matrices." << endl;
            cout << "You can choose your input type 'forward' of 'inverse' first and then type in your inputs, or you can type them in one line." << endl << endl;
            cout << "If you want to quit, press ctrl C and then type 'q' in the terminal." << endl;
            cout << "*******************************************************************************" << endl;
            cout << "Instruction : ";
            cin >> type ;
            switch(type) {
                case 'f':
                    cout << "Control joints (theta1~theta6) (degree): ";
                    cin >> joint1.data >> joint2.data >> joint3.data >> joint4.data >> joint5.data >> joint6.data;
                    
                    joint1.data = joint1.data / 180 * PI;
                    joint2.data = joint2.data / 180 * PI;
                    joint3.data = joint3.data / 180 * PI;
                    joint4.data = joint4.data / 180 * PI;
                    joint5.data = joint5.data / 180 * PI;
                    joint6.data = joint6.data / 180 * PI;
                    
                    break;

                case 'i':
                    cout << "input qx, qy ,qz : ";
                    cin >> qx >> qy >> qz;
                    cout << "Rx Ry Rz (degree): ";
                    cin >> rx >> ry >> rz;

                    rx = rx /180 *PI;
                    ry = ry /180 *PI;
                    rz = rz /180 *PI;
                    rotation = double_multi(Rx(rx), Ry(ry));
                    rotation = double_multi(rotation, Rz(rz));
                    //print_mat(rotation);

                    r11 = rotation[0][0];
                    r12 = rotation[0][1];
                    r13 = rotation[0][2];
                    r21 = rotation[1][0];
                    r22 = rotation[1][1];
                    r23 = rotation[1][2];
                    r31 = rotation[2][0];
                    r32 = rotation[2][1];
                    r33 = rotation[2][2];

                    px = qx - d6 * r13;            // Wrist position
                    py = qy - d6 * r23;
                    pz = qz - d6 * r33;

                    alpha = atan2(-d2, d4);        // Calculate theta1 by geometric method
                    beta = atan2(py, px);
                    theta1 = beta - alpha;


                    Wx = px - d2 * sin(theta1);    // Find the projection of the wrist on the surface formed by theta1 direction and z axis 
                    Wy = py + d2 * cos(theta1);
                    Wz = pz;

  
                    rxy = sqrt(Wx * Wx + Wy * Wy); // Length with respect to theta1 ditection
                    rxyz = sqrt((Wz - height) * (Wz - height) + rxy * rxy);  // Length from first joint to point W 
                    a = acos((rxyz * rxyz + a2 * a2 - d4 * d4) / (2 * rxyz * a2));  // Calculate theta2 and theta3 by geometric method
                    b = acos((d4 * d4 + a2 * a2 - rxyz * rxyz) / (2 * d4 * a2));
                    c = acos((d4 * d4 + rxyz * rxyz - a2 * a2) / (2 * d4 * rxyz));
                    theta2 = -(PI / 2 - a - atan2(Wz - height, rxy));
                    theta3 = -(PI / 2 - b);

                    // Transformation matrix
                    T[0][0] = r11;
                    T[0][1] = r12;
                    T[0][2] = r13;
                    T[0][3] = px;
                    T[1][0] = r21;
                    T[1][1] = r22;
                    T[1][2] = r23;
                    T[1][3] = py;
                    T[2][0] = r31;
                    T[2][1] = r32;
                    T[2][2] = r33;
                    T[2][3] = pz;
                    T[3][0] = 0;
                    T[3][1] = 0;
                    T[3][2] = 0;
                    T[3][3] = 1;

                    inv1 = inverse(theta1, 0, 0, alpha1);
                    inv2 = inverse((theta2 + PI / 2), d2, a2, alpha2);
                    inv3 = inverse(theta3, 0, 0, alpha3); 
                    tem1 = double_multi(inv1, T);
                    tem2 = double_multi(inv2, tem1);
                    T_3_6 = double_multi(inv3, tem2);

                    // Calculate theta4 to theta6
                    theta4 = atan2(T_3_6[1][2], T_3_6[0][2]);
                    theta5 = atan2(T_3_6[1][2] / sin(theta4), T_3_6[2][2]);
                    theta6 = atan2(T_3_6[2][1]/sin(theta5), -T_3_6[2][0]/sin(theta5));

                    printf("\ntheta1 = %f\ntheta2 = %f\ntheta3 = %f\ntheta4 = %f\ntheta5 = %f\ntheta6 = %f\n", theta1, theta2, theta3, theta4, theta5, theta6);
                    
                    
                    
                    A_0_1 = forward(theta1, 0, 0, alpha1);
                    A_1_2 = forward((theta2 + PI / 2), d2, a2, alpha2);
                    A_2_3 = forward(theta3, 0, 0, alpha3);
                    A_3_4 = forward(theta4, d4, 0, alpha4);
                    A_4_5 = forward(theta5, 0, 0, alpha5);
                    A_5_6 = forward(theta6, d6, 0, alpha6);

                    mul1 = double_multi(A_0_1, A_1_2);
                    A_0_3 = double_multi(mul1, A_2_3);
                    mul2 = double_multi(A_0_3, A_3_4);
                    A_0_5 = double_multi(mul2, A_4_5);
                    A_0_6 = double_multi(A_0_5, A_5_6);

                    A_0_6[2][3] += height;

                    printf("\nForward Kinematics :\n");
                    print_mat(A_0_6);
                    printf("\n");

                    joint1.data = theta1;
                    joint2.data = theta2;
                    joint3.data = theta3;
                    joint4.data = theta4;
                    joint5.data = theta5;
                    joint6.data = theta6;

                    break;
                
                case 'q':
                    cout << "Quit sj605 instruction." << endl;
                    //break;

                //default:
                    
                    
            }



            joint1_pub.publish(joint1);
            joint2_pub.publish(joint2);
            joint3_pub.publish(joint3);
            joint4_pub.publish(joint4);
            joint5_pub.publish(joint5);
            joint6_pub.publish(joint6);


      //      ROS_INFO("init: %d", init);
     /*  }*/


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

vector<vector<double> > double_multi(vector<vector<double> > A, vector<vector<double> > B)
{
    vector<vector<double> > C;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row;
    temp_row.resize(num_columns);
    for (unsigned int i = 0; i < num_rows; i++)
    {
        C.push_back(temp_row);
    }

    //printf("double_multiplication\n");
        
    int i, j, k;

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            C[i][j] = 0; 
            for (k = 0; k < 4; k++)
            {
                C[i][j] = C[i][j] + A[i][k] * B[k][j]; 
            }
            //printf("%lf ", C[i][j]); 
        }
        //printf("\n");
    }

    return C;

}

vector<vector<double> > inverse(double theta, double d, double a, double alpha)
{
    vector<vector<double> > M;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row;
    temp_row.resize(num_columns);
    for (unsigned int i = 0; i < num_rows; i++)
    {
        M.push_back(temp_row);
    }
    
    M[0][0] = cos(theta);
    M[0][1] = sin(theta);
    M[0][2] = 0;
    M[0][3] = -a;
    M[1][0] = -cos(alpha) * sin(theta);
    M[1][1] = cos(alpha) * cos(theta);
    M[1][2] = sin(alpha);
    M[1][3] = -d * sin(alpha);
    M[2][0] = sin(alpha) * sin(theta);
    M[2][1] = -sin(alpha) * cos(theta);
    M[2][2] = cos(alpha);
    M[2][3] = -d * cos(alpha);
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;

    return M;

}

vector<vector<double> > forward(double theta, double d, double a, double alpha)
{
    vector<vector<double> > M;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row;
    temp_row.resize(num_columns);
    for (unsigned int i = 0; i < num_rows; i++)
    {
        M.push_back(temp_row);
    }
    
    M[0][0] = cos(theta);
    M[0][1] = -cos(alpha) * sin(theta);
    M[0][2] = sin(alpha) * sin(theta);
    M[0][3] = a * cos(theta);
    M[1][0] = sin(theta);
    M[1][1] = cos(alpha) * cos(theta);
    M[1][2] = -sin(alpha) * cos(theta);
    M[1][3] = a * sin(theta);
    M[2][0] = 0;
    M[2][1] = sin(alpha);
    M[2][2] = cos(alpha);
    M[2][3] = d;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;

    return M;

}

void print_mat(vector<vector<double> > A)
{
    int i, j ;

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            printf("%lf ", A[i][j]); //輸出陣列C
        }
        printf("\n");
    }
}

vector<vector<double> > Rx(double theta)
{
    vector<vector<double> > M;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row;
    temp_row.resize(num_columns);
    for (unsigned int i = 0; i < num_rows; i++)
    {
        M.push_back(temp_row);
    }
    
    M[0][0] = 1;
    M[0][1] = 0;
    M[0][2] = 0;
    M[0][3] = 0;
    M[1][0] = 0;
    M[1][1] = cos(theta);
    M[1][2] = -sin(theta);
    M[1][3] = 0;
    M[2][0] = 0;
    M[2][1] = sin(theta);
    M[2][2] = cos(theta);
    M[2][3] = 0;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;
    return M;

}

vector<vector<double> > Ry(double theta)
{
    vector<vector<double> > M;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row;
    temp_row.resize(num_columns);
    for (unsigned int i = 0; i < num_rows; i++)
    {
        M.push_back(temp_row);
    }
    
    M[0][0] = cos(theta);
    M[0][1] = 0;
    M[0][2] = sin(theta);
    M[0][3] = 0;
    M[1][0] = 0;
    M[1][1] = 1;
    M[1][2] = 0;
    M[1][3] = 0;
    M[2][0] = -sin(theta);
    M[2][1] = 0;
    M[2][2] = cos(theta);
    M[2][3] = 0;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;

    return M;

}

vector<vector<double> > Rz(double theta)
{
    vector<vector<double> > M;
    unsigned int num_rows = 4, num_columns = 4;
    vector<double> temp_row;
    temp_row.resize(num_columns);
    for (unsigned int i = 0; i < num_rows; i++)
    {
        M.push_back(temp_row);
    }
    
    M[0][0] = cos(theta);
    M[0][1] = -sin(theta);
    M[0][2] = 0;
    M[0][3] = 0;
    M[1][0] = sin(theta);
    M[1][1] = cos(theta);
    M[1][2] = 0;
    M[1][3] = 0;
    M[2][0] = 0;
    M[2][1] = 0;
    M[2][2] = 1;
    M[2][3] = 0;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;



    return M;

}
