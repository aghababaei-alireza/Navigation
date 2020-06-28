#include <motion_planning/Map2D.h>

Map2D::Map2D(){
    resolution = 0.0;
    width = 0;
    height = 0;
    origin.Reset();
    data = std::vector< std::vector<int> >();
}

Map2D::Map2D(nav_msgs::OccupancyGrid map){
    Set(map);
}

void Map2D::Set(nav_msgs::OccupancyGrid map){
    resolution = map.info.resolution;
    width = map.info.width;
    height = map.info.height;
    origin.Set(
        map.info.origin.position.x,
        map.info.origin.position.y,
        getYawFromQuaternion(map.info.origin.orientation)
    );

    data.resize(height);
    for (int j = 0; j < height; j++)
    {
        data[j].resize(width);
    }

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            data[j][i] = map.data[i + width * j];
        }
    }
}

double Map2D::getResolution(){
    return resolution;
}

int Map2D::getWidth(){
    return width;
}

int Map2D::getHeight(){
    return height;
}

double Map2D::getYawFromQuaternion(geometry_msgs::Quaternion q){
    tf::Quaternion Q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(Q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

navigation_msgs::Vector3 Map2D::getOrigin(){
    return origin;
}