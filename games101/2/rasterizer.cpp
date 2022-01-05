// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>


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

static bool insideTriangle(float x, float y, const Vector4f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f a(_v[0].x(), _v[0].y(), 0.0f);
    Eigen::Vector3f b(_v[1].x(), _v[1].y(), 0.0f);
    Eigen::Vector3f c(_v[2].x(), _v[2].y(), 0.0f);
    Eigen::Vector3f p(x, y, 0.0f);

    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f bc = c - b;
    Eigen::Vector3f ca = a - c;
    Eigen::Vector3f ap = p - a;
    Eigen::Vector3f bp = p - b;
    Eigen::Vector3f cp = p - c;

    Eigen::Vector3f d1 = ap.cross(ab);
    Eigen::Vector3f d2 = bp.cross(bc);
    Eigen::Vector3f d3 = cp.cross(ca);

    if (d1.dot(d2) > 0 && d1.dot(d3) > 0)  //三个向量应该同向
        return true;
    else
        return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::GetBoundingBox(Eigen::Vector2i &min, Eigen::Vector2i &max, const Triangle& t)
{
    float min_x = std::min(t.v[0].x(), std::min(t.v[1].x(), t.v[2].x()));
    float max_x = std::max(t.v[0].x(), std::max(t.v[1].x(), t.v[2].x()));

    float min_y = std::min(t.v[0].y(), std::min(t.v[1].y(), t.v[2].y()));
    float max_y = std::max(t.v[0].y(), std::max(t.v[1].y(), t.v[2].y()));

    min.x() = static_cast<int>(min_x);
    min.y() = static_cast<int>(min_y);

    max.x() = std::clamp(static_cast<int>(std::ceil(max_x)), 0, width - 1);
    max.y() = std::clamp(static_cast<int>(std::ceil(max_y)), 0, height - 1);
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;  //这里的near/far取值为0.1/50，Opengl的视口变换默认的near/far取值为0/1
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
        for (auto& vec : v) {  // 透视除法，注意这里保留了vec.w()没有化为1，vec.w()的值等于相机空间的(-eye.z)
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
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
            t.setVertex(i, v[i]);
            t.setVertex(i, v[i]);
            t.setVertex(i, v[i]);
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

float GetViewSpaceZ(float alpha, float beta, float gamma, const Eigen::Vector3f &viewspace_triange_z)
{
    return 1.0 / (alpha / viewspace_triange_z[0] + beta / viewspace_triange_z[1] + gamma / viewspace_triange_z[2]);
}

float GetAttr(float alpha, float beta, float gamma, float viewspace_z, const Eigen::Vector3f &viewspace_triange_z, const Eigen::Vector3f &attr)
{
    return viewspace_z * (alpha*attr[0] / viewspace_triange_z[0] +
                          beta*attr[1] / viewspace_triange_z[1] +
                          gamma*attr[2] / viewspace_triange_z[2]);
}

//Screen space rasterization  参考：透视纠正插值 https://zhuanlan.zhihu.com/p/45757899
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    Eigen::Vector2i min, max;
    GetBoundingBox(min, max, t); //计算三角形的包围盒

    // printf("%d,%d %d,%d \n", min.x(), min.y(), max.x(), max.y());

    for (int x = min.x(); x < max.x(); ++x) {
        for (int y = min.y(); y < max.y(); ++y) {
            Eigen::Vector3f point(x + 0.5, y + 0.5, 0.0f); // +0.5以便取像素的中心位置
            if (!insideTriangle(point.x(), point.y(), t.v))
                continue;

            // 计算点point在2D屏幕空间下的重心坐标
            float alpha, beta, gamma;
            std::tie(alpha, beta, gamma) = computeBarycentric2D(point.x(), point.y(), t.v);

            Eigen::Vector3f viewspace_triange_z(t.v[0].w(), t.v[1].w(), t.v[2].w());
            Eigen::Vector3f screenspace_triange_z(t.v[0].z(), t.v[1].z(), t.v[2].z());
            // 计算点point在VIEW SPACE下的Z值
            float viewspace_z = GetViewSpaceZ(alpha, beta, gamma, viewspace_triange_z); 
            // 计算NDC下的Z值，这里是把NDC下的Z值当做属性看待，然后进行线性插值
            float z_buffer = GetAttr(alpha, beta, gamma, viewspace_z, viewspace_triange_z, screenspace_triange_z);

            if (z_buffer < depth_buf[get_index(x, y)]) {
                depth_buf[get_index(x, y)] = z_buffer;

                // 属性插值
                Eigen::Vector3f color_r(t.color[0][0], t.color[1][0], t.color[2][0]);
                Eigen::Vector3f color_g(t.color[0][1], t.color[1][1], t.color[2][1]);
                Eigen::Vector3f color_b(t.color[0][2], t.color[1][2], t.color[2][2]);
                Eigen::Vector3f color;
                color[0] = GetAttr(alpha, beta, gamma, viewspace_z, viewspace_triange_z, color_r);
                color[1] = GetAttr(alpha, beta, gamma, viewspace_z, viewspace_triange_z, color_g);
                color[2] = GetAttr(alpha, beta, gamma, viewspace_z, viewspace_triange_z, color_b);

                set_pixel(point, color);
            }
        }
    }

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
    int ind = (height-1-(int)point.y())*width + (int)point.x();
    frame_buf[ind] = color;

}

// clang-format on