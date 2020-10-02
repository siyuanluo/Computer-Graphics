#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
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
    float rad = rotation_angle / 180 * acos(-1);
    model << cos(rad),-sin(rad),0,0,
             sin(rad),cos(rad),0,0,
             0,0,1,0,
             0,0,0,1;  // rotate around Z axis
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float rad = eye_fov / 180 * acos(-1);
    float t = abs(zNear) * tan(rad/2.0);
    float b = -t, r = t * aspect_ratio;
    float l = -r;
    Eigen::Matrix4f scale, translate, pertoorth;
    pertoorth << zNear,0,0,0,
            0,zNear,0,0,
            0,0,zNear+zFar,-zNear*zFar,
            0,0,1,0;
    // don't forget this matrix;
    translate << 1,0,0,-(r+l)/2.0,
                 0,1,0,-(t+b)/2.0,
                 0,0,1,-(zNear+zFar)/2.0,
                 0,0,0,1;
    scale << 2.0/(r-l),0,0,0,
             0,2.0/(t-b),0,0,
             0,0,2.0/(zNear-zFar),0,
             0,0,0,1;
    projection = pertoorth * translate * scale;
    return projection;
}

// this is the bonus question, and I don't know if it's correct
Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float rad = angle / 180 * acos(-1);
    float unit = sqrt(axis(0) * axis(0)+axis(1) * axis(1)+axis(2) * axis(2));
    axis(0) /= unit;
    axis(1) /= unit;
    axis(2) /= unit;
    // unitization
    Eigen::Vector3f axisT = axis.adjoint();
    Eigen::Matrix4f firstM, secondM, thirdM;
    firstM << cos(rad),0,0,
              0,cos(rad),0,
              0,0,cos(rad);
    secondM = axis * axisT * (1-cos(rad));
    thirdM << 0,-aixs(2),axis(1),
              axis(2),0,-axis(0),
              -axis(1),axis(0),0;
    thirdM *= sin(rad);
    model = firstM + secondM + thirdM;
    // rotate around axis

    return model;
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
