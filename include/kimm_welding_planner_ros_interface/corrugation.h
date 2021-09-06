#include <Eigen/Dense>
#include <kimm_trajectory_smoother/Trajectory.h>
#include <kimm_trajectory_smoother/Path.h>

using namespace Eigen;

class Corrugation_shape{
    public:
        Corrugation_shape(const double & radius, const double & width, const double & height, const Vector3d & center, const unsigned int & type);
        ~Corrugation_shape(){};

        void set_normal(const Vector3d & normal);
        void extend(kimmtraj::stdlist_Eigenvec & waypoint);
        Eigen::Vector3d get_projected_normal();
        void generate_ellipsoid();
        void generate_circle(const unsigned int & index);

    protected:
        double a_, b_;
        double r_;
        double w_;
        double h_;
        Vector3d center_, normal_, c_normal_, rot_axis_, init_normal_;
        Vector3d direction_;
        unsigned int type_;
        int r_interval_, e_interval_;
        std::vector<Vector4d> path_;
        Vector4d point_;
        AngleAxisd rot_; 
};
