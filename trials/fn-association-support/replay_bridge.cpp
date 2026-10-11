// 동일한 Camera optical point와 timestamp로 수정 전후 C++ association·tracker를 비교한다.
// 좌표 단위는 meter이며 GT는 이 binding에 전달하지 않는다.
// FN_BASELINE_ASSOCIATION을 정의하면 보존한 수정 전 association header를 사용한다.
#ifdef FN_BASELINE_ASSOCIATION
#include "baseline-association.hpp"
#endif
#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"
#include "mobile_robot_lab_cpp/constant_velocity_tracker.hpp"
using namespace mobile_robot_lab_cpp;
extern "C" {
int assoc(double* p,int n,double* out) {
 std::vector<Point3> pts;
 for(int i=0;i<n;i++) pts.push_back({p[i*3],p[i*3+1],p[i*3+2]});
 const Rotation3 r{{{{1,0,0}},{{0,1,0}},{{0,0,1}}}};
 auto a=analyze_instance_points(pts,r,{0,0,0});
 if(!a.association) return static_cast<int>(a.status);
 out[0]=a.association->position_camera.x; out[1]=a.association->position_camera.y;
 out[2]=a.association->position_camera.z; out[3]=a.association->point_count;
 return 0;
}
void* tracker_create(){return new ConstantVelocityTracker();}
void tracker_free(void* t){delete static_cast<ConstantVelocityTracker*>(t);}
int tracker_update(void* t,double* p,int n,double stamp,double* out) {
 std::vector<Detection2> detections;
 for(int i=0;i<n;i++) detections.push_back({p[i*2],p[i*2+1]});
 const auto& ts=static_cast<ConstantVelocityTracker*>(t)->update(detections,stamp);
 if(ts.size()>1000) return -1;
 for(std::size_t i=0;i<ts.size();i++){out[i*3]=ts[i].id; out[i*3+1]=ts[i].estimate[0];out[i*3+2]=ts[i].estimate[1];}
 return ts.size();
}
}
