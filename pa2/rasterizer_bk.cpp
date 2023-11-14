// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f AB({v[1].x()- v[0].x(),v[1].y() - v[0].y()});
    Eigen::Vector2f BC({v[2].x()- v[1].x(),v[2].y() - v[1].y()});
    Eigen::Vector2f CA({v[0].x()- v[2].x(),v[0].y() - v[2].y()});
    Eigen::Vector2f AP({x- v[0].x(),y - v[0].y()});
    Eigen::Vector2f BP({x- v[1].x(),y - v[1].y()});
    Eigen::Vector2f CP({x- v[2].x(),y - v[2].y()});

    
    auto crossABAP = AB(0) * AP(1) - AB(1) * AP(0);
    auto crossBCBP = BC(0) * BP(1) - BC(1) * BP(0);
    auto crossCACP = CA(0) * CP(1) - CA(1) * CP(0);

    if((crossABAP >=0 and crossBCBP >=0 and crossCACP >=0) or (crossABAP <0 and crossBCBP <0 and crossCACP <0))
    {
        return true;
    }

    return false;

}



static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    Eigen::Vector4f p1 = v[0];
    Eigen::Vector4f p2 = v[1];
    Eigen::Vector4f p3 = v[2];

    auto p1x = p1(0);
    auto p1y = p1(1);
    auto p2x = p2(0);
    auto p2y = p2(1);
    auto p3x = p3(0);
    auto p3y = p3(1);




    auto leftx = std::min(std::min(p1x,p2x),p3x);
    auto rightx = std::max(std::max(p1x,p2x),p3x);

    auto bottomy = std::min(std::min(p1y,p2y),p3y);
    auto topy = std::max(std::max(p1y,p2y),p3y);

    // int xMin, yMin, xMax, yMax;
    // xMin = std::floor(std::min(std::min(v[0].x(),v[1].x()),v[2].x()));
    // yMin = std::floor(std::min(std::min(v[0].y(), v[1].y()), v[2].y()));
    // xMax = std::ceil(std::max(std::max(v[0].x(), v[1].x()), v[2].x()));
    // yMax = std::ceil(std::max(std::max(v[0].y(), v[1].y()), v[2].y()));


    // iterate through the pixel and find if the current pixel is inside the triangle

    // for (int i = xMin; i <= xMax; i++)
    // {
    //     for (int j = yMin; j <= yMax; j++)
    //     {
    //         if (insideTriangle(i+0.5f, j+0.5f,t.v))
    //         {
    //             auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
    //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;
    //             int index = get_index(i, j);
    //             if (depth_buf[index] > z_interpolated)
    //             {
    //                 depth_buf[index] = z_interpolated; // 更新深度缓冲区
    //                 set_pixel(Vector3f(i,j,z_interpolated),t.getColor());
    //             }
    //         }
    //     }
    // }

    for (int i = leftx;i < rightx;i++)
    {
        for (int j = bottomy; j < topy; j++)
        {
            int num = 0;
            if(insideTriangle(i+ 0.25f,j+0.25f,t.v)) num++;
            if(insideTriangle(i+ 0.75f,j+0.75f,t.v)) num++;
            if(insideTriangle(i+ 0.25f,j+0.75f,t.v)) num++;
            if(insideTriangle(i+ 0.75f,j+0.25f,t.v)) num++;
            if(num > 0)
            {
                int index = get_index(i,j);
                std::tuple<float, float, float> depw = computeBarycentric2D(i,j,t.v);
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (z_interpolated < depth_buf[index]) {
                    depth_buf[index] = z_interpolated;
                    set_pixel(Vector3f(i,j,z_interpolated),t.getColor() * num / 4);
                };

            }
        }
    }


    // If so, use the following code to get the interpolated z value

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on