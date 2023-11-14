#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float angle = rotation_angle /180.0*acos(-1);
    Eigen::Matrix4f rotationByZ;
    rotationByZ << std::cos(angle),-std::sin(angle),0.f,0.f,
                    std::sin(angle),std::cos(angle),0.f,0.f,
                    0.f,0.f,1.f,0.f,
                    0.f,0.f,0.f,1.f;
    model =  rotationByZ * model;

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // eye_fov x 方向视角， aspect_ratio 宽高比

    float fovy = eye_fov;
    float fovx = fovy * aspect_ratio;

    // P = | 1/tan(fovy/2)     0             0                     0               |
    //     |       0       1/tan(fovx/2)     0                     0               |
    //     |       0            0       -(f+n)/(f-n)    -2*f*n/(f-n)               |
    //     |       0            0             -1                    0              |


    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float item1 = 1.0f / std::tan(fovy / 2.0f);
    float item2 = 1.0f / std::tan(fovx / 2.0f);
    float item3 = - (zFar + zNear) / (zFar - zNear);
    float item4 = - 2 * zFar * zNear / ( zFar - zNear); 

    projection << item1,0.f,0.f,0.f,
                    0.f,item2,0.f,0.f,
                    0.f,0.f,item3,item4,
                    0.f,0.f,-1.0f,0.f;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    Eigen::Matrix3f matrixK;
    matrixK << 0.f,-axis(2),axis(1),
                axis(2),0.f,-axis(0),
                -axis(1),axis(0),0.f;
    
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    angle = angle / 180 * std::acos(-1);
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    rotation = std::cos(angle) * I + (1 - std::cos(angle)) * axis * axis.transpose() + std::sin(angle) * matrixK;
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block(0,0,3,3) << rotation;
    // std::cout << result << '\n';
    return result;

}

int main(int argc, const char** argv)
{
    
    
    
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
