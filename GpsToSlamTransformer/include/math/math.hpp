#ifndef MATH__HPP
#define MATH__HPP

#include "define_container/define_container.hpp"
#include "proj.h"
#include "proj_api.h"

namespace gps_to_slam_transformer
{

    class SLAMPoint final
    {
    private:
        double x_;
        double y_;

    public:
        inline explicit SLAMPoint()
        {
        }

        inline explicit SLAMPoint(const double &x, const double &y)
        {
            this->x_ = x;
            this->y_ = y;
        }

        inline virtual ~SLAMPoint()
        {
        }

        inline double get__x()
        {
            return this->x_;
        }

        inline void set__x(const double &x)
        {
            this->x_ = x;
        }

        inline double get__y()
        {
            return this->y_;
        }

        inline void set__y(const double &y)
        {
            this->y_ = y;
        }
    };

    class KTMPoint final
    {
    private:
        double x_;
        double y_;

    public:
        inline explicit KTMPoint()
        {
        }

        inline virtual ~KTMPoint()
        {
        }

        inline double get__x()
        {
            return this->x_;
        }

        inline void set__x(const double &x)
        {
            this->x_ = x;
        }

        inline double get__y()
        {
            return this->y_;
        }

        inline void set__y(const double &y)
        {
            this->y_ = y;
        }
    };

    class GTSPoint final
    {
    private:
        std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_1_ptr_;
        std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_2_ptr_;

        std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_1_ptr_;
        std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_2_ptr_;

    public:
        inline explicit GTSPoint()
        {
            slam_point_1_ptr_ = std::make_shared<gps_to_slam_transformer::SLAMPoint>();
            slam_point_2_ptr_ = std::make_shared<gps_to_slam_transformer::SLAMPoint>();

            ktm_point_1_ptr_ = std::make_shared<gps_to_slam_transformer::KTMPoint>();
            ktm_point_2_ptr_ = std::make_shared<gps_to_slam_transformer::KTMPoint>();
        }

        inline virtual ~GTSPoint()
        {
        }

        inline std::shared_ptr<gps_to_slam_transformer::SLAMPoint> get__slam_point_1_ptr()
        {
            return this->slam_point_1_ptr_;
        }

        inline void set__slam_point_1_ptr(std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_1_ptr)
        {
            this->slam_point_1_ptr_ = slam_point_1_ptr;
        }

        inline std::shared_ptr<gps_to_slam_transformer::SLAMPoint> get__slam_point_2_ptr()
        {
            return this->slam_point_2_ptr_;
        }

        inline void set__slam_point_2_ptr(std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_2_ptr)
        {
            this->slam_point_2_ptr_ = slam_point_2_ptr;
        }

        inline std::shared_ptr<gps_to_slam_transformer::KTMPoint> get__ktm_point_1_ptr()
        {
            return this->ktm_point_1_ptr_;
        }

        inline void set__ktm_point_1_ptr(std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_ptr)
        {
            this->ktm_point_1_ptr_ = ktm_point_ptr;
        }

        inline std::shared_ptr<gps_to_slam_transformer::KTMPoint> get__ktm_point_2_ptr()
        {
            return this->ktm_point_2_ptr_;
        }

        inline void set__ktm_point_2_ptr(std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_ptr)
        {
            this->ktm_point_2_ptr_ = ktm_point_ptr;
        }
    };

    class QuaternionPoint final
    {
    private:
        double x_, y_, z_, w_;
        double roll_, pitch_, yaw_;
        double qr_distance_;
        double height_;
        double base_line_;
        double double_radian_;
        double euler_x_;
        double euler_y_;
        double euler_z_;
        double euler_w_;

    public:
        inline explicit QuaternionPoint()
            : x_(RCL_DEFAULT_DOUBLE),
              y_(RCL_DEFAULT_DOUBLE),
              z_(RCL_DEFAULT_DOUBLE),
              w_(RCL_DEFAULT_DOUBLE),
              roll_(RCL_DEFAULT_DOUBLE),
              pitch_(RCL_DEFAULT_DOUBLE),
              yaw_(RCL_DEFAULT_DOUBLE),
              qr_distance_(RCL_DEFAULT_DOUBLE),
              height_(RCL_DEFAULT_DOUBLE),
              base_line_(RCL_DEFAULT_DOUBLE),
              double_radian_(RCL_DEFAULT_DOUBLE),
              euler_x_(RCL_DEFAULT_DOUBLE),
              euler_y_(RCL_DEFAULT_DOUBLE),
              euler_z_(RCL_DEFAULT_DOUBLE),
              euler_w_(RCL_DEFAULT_DOUBLE)
        {
        }

        inline virtual ~QuaternionPoint()
        {
        }

        inline void quaternion_to_euler_angle()
        {
            double sinr_cosp = 2 * (w_ * x_ + y_ * z_);
            double cosr_cosp = 1 - 2 * (x_ * x_ + y_ * y_);

            roll_ = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = std::sqrt(1 + 2 * (w_ * y_ - x_ * z_));
            double cosp = std::sqrt(1 - 2 * (w_ * y_ - x_ * z_));

            pitch_ = 2 * std::atan2(sinp, cosp) - M_PI / 2;

            if (pitch_ == 1.5708)
            {
                pitch_ = 1.5918;
            }
            else if (pitch_ == -1.5708)
            {
                pitch_ = -1.5918;
            }

            double siny_cosp = 2 * (w_ * z_ + x_ * y_);
            double cosy_cosp = 1 - 2 * (y_ * y_ + z_ * z_);

            yaw_ = std::atan2(siny_cosp, cosy_cosp);
        }

        inline double get__roll()
        {
            return this->roll_;
        }

        inline double get__pitch()
        {
            return this->pitch_;
        }

        inline double get__yaw()
        {
            return this->yaw_;
        }

        inline double get__x()
        {
            return (float)this->x_;
        }

        inline void set__x(const double &x)
        {
            this->x_ = x;
        }

        inline double get__y()
        {
            return (float)this->y_;
        }

        inline void set__y(const double &y)
        {
            this->y_ = y;
        }

        inline double get__z()
        {
            return (float)this->z_;
        }

        inline void set__z(const double &z)
        {
            this->z_ = z;
        }

        inline double get__w()
        {
            return (float)this->w_;
        }

        inline void set__w(const double &w)
        {
            this->w_ = w;
        }

        inline double degree_to_radian(double degree)
        {
            double_radian_ = degree * M_PI / 180;
            return double_radian_;
        }

        inline double get__height()
        {
            height_ = sin(double_radian_) * qr_distance_;
            return height_;
        }

        inline double get__base_line()
        {
            double temp_height = height_;
            double temp_qr_distance = qr_distance_;

            temp_height = pow(temp_height, 2);
            temp_qr_distance = pow(temp_qr_distance, 2);

            base_line_ = temp_height + temp_qr_distance;
            base_line_ = sqrt(base_line_);

            return base_line_;
        }

        inline void euler_angle_to_quaternion(double yaw, double pitch, double roll)
        {
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);
            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);

            euler_x_ = sr * cp * cy - cr * sp * sy;

            euler_y_ = cr * sp * cy + sr * cp * sy;

            euler_z_ = cr * cp * sy - sr * sp * cy;

            euler_w_ = cr * cp * cy + sr * sp * sy;
        }

        inline double get__euler_w()
        {
            return euler_w_;
        }

        inline double get__euler_z()
        {
            return euler_z_;
        }

        inline double get__euler_x()
        {
            return euler_x_;
        }

        inline double get_euler_y()
        {
            return euler_y_;
        }
    };

    /**
     * @class gps_to_slam_transformer::CommonMath
     * @brief final class for implements calculating functions
     */
    class CommonMath final
    {
    public:
        /**
         * create a new this class' instance
         * @brief inline default constructor
         */
        inline explicit CommonMath()
        {
        }

        /**
         * destroy this class' instance
         * @brief inline default destructor
         */
        inline virtual ~CommonMath()
        {
        }

        inline double distance_formula(double x1, double y1, double x2, double y2)
        {
            const double &sqr_x = (x2 - x1) * (x2 - x1);
            const double &sqr_y = (y2 - y1) * (y2 - y1);

            double distance = sqrt(sqr_x + sqr_y);

            return distance;
        }

        inline double calculate_radian(double x1, double y1, double x2, double y2)
        {
            const double &atan_x = x2 - x1;
            const double &atan_y = y2 - y1;

            double radian = atan2(atan_y, atan_x);

            return radian;
        }

        /**
         * @brief inline function for transform degrees to radian
         * @param degrees double
         * @return radian double
         */
        inline double degrees_to_radian(double degrees)
        {
            double radian = degrees * M_PI / 180.0;

            RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "calculated - degrees to radian : [%f]", radian);
            RCLCPP_LINE_INFO();

            return radian;
        }

        inline double radian_to_degrees(double radians)
        {
            double radian_degrees = (radians * (M_PI / 180.0));

            return radian_degrees;
        }

        inline double angle_between_two_points(double x1, double x2, double y1, double y2)
        {
            const double &atan_x = x2 - x1;
            const double &atan_y = y2 - y1;

            double radian = atan2(atan_y, atan_x);

            return radian;
        }
    };

    class GpsMath final
    {
    private:
        std::shared_ptr<gps_to_slam_transformer::CommonMath> common_math_ptr_;

    public:
        inline explicit GpsMath()
        {
            common_math_ptr_ = std::make_shared<gps_to_slam_transformer::CommonMath>();
        }

        inline virtual ~GpsMath()
        {
        }

        inline double distance_formula_kilometers(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
        {
            const double &phi_1 = common_math_ptr_->degrees_to_radian(latitude_1);
            const double &phi_2 = common_math_ptr_->degrees_to_radian(latitude_2);
            const double &delta_phi = common_math_ptr_->degrees_to_radian(latitude_2 - latitude_1);
            const double &delta_lambda = common_math_ptr_->degrees_to_radian(longitude_2 - longitude_1);

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "calculated - degrees to kilometers\n\tphi_1 : [%f]\n\tphi_2 : [%f]\n\tdelta_phi : [%f]\n\tdelta_lambda : [%f]",
                phi_1,
                phi_2,
                delta_phi,
                delta_lambda);
            RCLCPP_LINE_INFO();

            double a = (sin(delta_phi / 2.0) * sin(delta_phi / 2.0)) + (cos(phi_1) * cos(phi_2) * sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0));
            double c = 2 * atan2(sqrt(a), sqrt(1 - a));

            double distance_km = EARTH_RADIUS_KTM * c;

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "calculated - degrees to kilometers\n\ta : [%f]\n\tc : [%f]distance_km : [%f]",
                a,
                c,
                distance_km);
            RCLCPP_LINE_INFO();

            return distance_km;
        }

        inline double distance_formula_meters(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
        {
            const double distance = (this->distance_formula_kilometers(latitude_1, longitude_1, latitude_2, longitude_2) * 1000);

            return distance;
        }

        double angle_between_two_points(double latitude_1, double longitude_1, double latitude_2, double longitude_2)
        {
            double phi_1 = latitude_1 * M_PI / 180;
            double phi_2 = latitude_2 * M_PI / 180;
            double lambda_1 = longitude_1 * M_PI / 180;
            double lambda_2 = longitude_2 * M_PI / 180;

            double y = sin(lambda_2 - lambda_1) * cos(phi_2);
            double x = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(lambda_2 - lambda_1);

            const double &angle = atan2(y, x);

            return angle;
        }
    };

    class KTMMath final
    {
    private:
    public:
        inline explicit KTMMath()
        {
        }

        inline virtual ~KTMMath()
        {
        }

        inline std::shared_ptr<gps_to_slam_transformer::KTMPoint> geo_point_to_ktm_point(double latitude, double longitude)
        {
            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "ktm math GtK raw\n\tlat : [%f]\n\tlon : [%f]",
                latitude,
                longitude);
            RCLCPP_LINE_INFO();

            PJ *P = proj_create_crs_to_crs(0, "EPSG:4326", "EPSG:5186", 0);

            PJ_COORD pj_coord;

            pj_coord.xy.x = latitude;
            pj_coord.xy.y = longitude;

            PJ_COORD re = proj_trans(P, PJ_FWD, pj_coord);

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "ktm math GtK calculated\n\tlat : [%f]n\tlon : [%f]",
                re.xy.y,
                re.xy.x);
            RCLCPP_LINE_INFO();

            pj_free(P);

            std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_point_ptr = std::make_shared<gps_to_slam_transformer::KTMPoint>();
            ktm_point_ptr->set__x(re.xy.y);
            ktm_point_ptr->set__y(re.xy.x);

            return ktm_point_ptr;
        }
    };

    class GTSMath final
    {
    private:
        std::shared_ptr<gps_to_slam_transformer::CommonMath> common_math_ptr_;

    public:
        inline explicit GTSMath()
        {
            common_math_ptr_ = std::make_shared<gps_to_slam_transformer::CommonMath>();
        }

        inline virtual ~GTSMath()
        {
        }

        inline double calculate_rotation(std::shared_ptr<gps_to_slam_transformer::GTSPoint> gts_point_ptr)
        {
            const double &x1 = gts_point_ptr->get__slam_point_1_ptr()->get__x();
            const double &y1 = gts_point_ptr->get__slam_point_1_ptr()->get__y();

            const double &x2 = gts_point_ptr->get__slam_point_2_ptr()->get__x();
            const double &y2 = gts_point_ptr->get__slam_point_2_ptr()->get__y();

            const double &x3 = gts_point_ptr->get__ktm_point_1_ptr()->get__x();
            const double &y3 = gts_point_ptr->get__ktm_point_1_ptr()->get__y();

            const double &x4 = gts_point_ptr->get__ktm_point_2_ptr()->get__x();
            const double &y4 = gts_point_ptr->get__ktm_point_2_ptr()->get__y();

            double v_ab_x = x2 - x1;
            double v_ab_y = y2 - y1;
            double v_cd_x = x4 - x3;
            double v_cd_y = y4 - y3;

            double dot_product = (v_ab_x * v_cd_x) + (v_ab_y * v_cd_y);

            const double &magnitude_ab = sqrt(v_ab_x * v_ab_x + v_ab_y * v_ab_y);
            const double &magnitude_cd = sqrt(v_cd_x * v_cd_x + v_cd_y * v_cd_y);

            const double &angle_radian = acos(dot_product / (magnitude_ab * magnitude_cd));

            return angle_radian;
        }
    };

    class SLAMMath final
    {
    private:
        std::shared_ptr<gps_to_slam_transformer::CommonMath> common_math_ptr_;

    public:
        inline explicit SLAMMath()
        {
            common_math_ptr_ = std::make_shared<gps_to_slam_transformer::CommonMath>();
        }

        inline virtual ~SLAMMath()
        {
        }

        inline double distance_formula(
            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> start_point_ptr,
            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> end_point_ptr)
        {
            const double &start_x = start_point_ptr->get__x();
            const double &start_y = start_point_ptr->get__y();

            const double &end_x = end_point_ptr->get__x();
            const double &end_y = end_point_ptr->get__y();

            const double &distance = common_math_ptr_->distance_formula(start_x, start_y, end_x, end_y);

            return distance;
        }

        inline double angle_between_two_points(
            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> start_point_ptr,
            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> end_point_ptr)
        {
            const double &start_x = start_point_ptr->get__x();
            const double &start_y = start_point_ptr->get__y();

            const double &end_x = end_point_ptr->get__x();
            const double &end_y = end_point_ptr->get__y();

            const double &angle = common_math_ptr_->angle_between_two_points(start_x, start_y, end_x, end_y);

            return angle;
        }

        inline std::shared_ptr<gps_to_slam_transformer::SLAMPoint> rotate_point(double x, double y, double angle)
        {
            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_ptr = std::make_shared<gps_to_slam_transformer::SLAMPoint>();

            double new_x = (x * cos(angle)) - (y * sin(angle));
            double new_y = (x * sin(angle)) - (y * cos(angle));

            slam_point_ptr->set__x(new_x);
            slam_point_ptr->set__y(new_y);

            return slam_point_ptr;
        }
    };

    class SLAMPointGenerator final
    {
    private:
        std::shared_ptr<gps_to_slam_transformer::CommonMath> common_math_ptr_;
        std::shared_ptr<gps_to_slam_transformer::GpsMath> gps_math_ptr_;
        std::shared_ptr<gps_to_slam_transformer::GTSMath> gts_math_ptr_;

    public:
        inline explicit SLAMPointGenerator()
        {
            common_math_ptr_ = std::make_shared<gps_to_slam_transformer::CommonMath>();
            gps_math_ptr_ = std::make_shared<gps_to_slam_transformer::GpsMath>();
            gts_math_ptr_ = std::make_shared<gps_to_slam_transformer::GTSMath>();
        }

        inline virtual ~SLAMPointGenerator()
        {
        }

        inline std::shared_ptr<gps_to_slam_transformer::SLAMPoint> geo_point_to_slam_point(
            std::shared_ptr<gps_to_slam_transformer::GTSPoint> gts_point_ptr,
            std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_des_point_ptr)
        {
            std::shared_ptr<gps_to_slam_transformer::SLAMPoint> slam_point_ptr = std::make_shared<gps_to_slam_transformer::SLAMPoint>();

            std::shared_ptr<gps_to_slam_transformer::KTMPoint> ktm_bcm_point_ptr = gts_point_ptr->get__ktm_point_1_ptr();

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "slam point generator GtS\n\tktm btm x : [%f]\n\tktm btm y : [%f]\n\tktm des x : [%f]\n\tktm des y : [%f]",
                ktm_bcm_point_ptr->get__x(),
                ktm_bcm_point_ptr->get__y(),
                ktm_des_point_ptr->get__x(),
                ktm_des_point_ptr->get__y());
            RCLCPP_LINE_INFO();

            const double &distance = common_math_ptr_->distance_formula(
                ktm_bcm_point_ptr->get__x(), ktm_bcm_point_ptr->get__y(),
                ktm_des_point_ptr->get__x(), ktm_des_point_ptr->get__y());

            const double &rotated_angle = gts_math_ptr_->calculate_rotation(gts_point_ptr);

            const double &start_ktm_to_des_radian = common_math_ptr_->calculate_radian(
                ktm_bcm_point_ptr->get__x(), ktm_bcm_point_ptr->get__y(),
                ktm_des_point_ptr->get__x(), ktm_des_point_ptr->get__y());

            const double &radian = start_ktm_to_des_radian + rotated_angle + M_PI;

            RCUTILS_LOG_INFO_NAMED(
                RCL_NODE_NAME,
                "slam point generate Gts\n\tdistance : [%f]\n\trotate angle : [%f]\n\tstart_ktm_to_des_radian : [%f]\n\tradian : [%f]",
                distance,
                rotated_angle,
                start_ktm_to_des_radian,
                radian);
            RCLCPP_LINE_INFO();

            const double &x = gts_point_ptr->get__slam_point_1_ptr()->get__x() + distance * cos(radian);
            const double &y = gts_point_ptr->get__slam_point_1_ptr()->get__y() + distance * sin(radian);

            slam_point_ptr->set__x(x);
            slam_point_ptr->set__y(y);

            return slam_point_ptr;
        }
    };
};

#endif