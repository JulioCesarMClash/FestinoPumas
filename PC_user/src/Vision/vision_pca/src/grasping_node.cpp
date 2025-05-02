#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

typedef pcl::PointXYZ PointT;

class ObjectSegmentation
{
    private:
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);
            latest_cloud_ = cloud;
        }

        ros::Subscriber sub_;
        ros::Publisher pub_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;

    public:
        ObjectSegmentation(ros::NodeHandle& nh)
        {
            sub_ = nh.subscribe("/camera/depth/points", 1, &ObjectSegmentation::cloudCallback, this);
            pub_ = nh.advertise<sensor_msgs::PointCloud2>("segmented_object", 1);
        }

        void processPointCloud()
        {
            if (!latest_cloud_) return;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*latest_cloud_));

            
            // 1. Filtrado por altura
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.0, 2.0);
            pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
            pass.filter(*cloud_filtered);
            
            // 2. Segmentar el plano
            pcl::SACSegmentation<PointT> seg;
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.03);
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            
            // 3. Eliminar el plano
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<PointT>::Ptr cloud_no_plane(new pcl::PointCloud<PointT>);
            extract.filter(*cloud_no_plane);

            // 4. Clustering
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            tree->setInputCloud(cloud_no_plane);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance(0.05);  // distancia máx. entre puntos del mismo clúster
            ec.setMinClusterSize(100);     // mínimo número de puntos por clúster
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_no_plane);
            ec.extract(cluster_indices);

            // 5. Extraer el primer clúster
            pcl::PointCloud<PointT>::Ptr object_cluster(new pcl::PointCloud<PointT>);
            if (!cluster_indices.empty())
            {
                pcl::ExtractIndices<PointT> extract_cluster;
                pcl::PointIndices::Ptr object_indices(new pcl::PointIndices(cluster_indices[4]));
                extract_cluster.setInputCloud(cloud_no_plane);
                extract_cluster.setIndices(object_indices);
                extract_cluster.setNegative(false);  // mantener solo el clúster
                extract_cluster.filter(*object_cluster);
            }
            else
            {
                ROS_WARN("No clusters found!");
                return;
            }


            pcl::PCA<PointT> pca;
            pca.setInputCloud(object_cluster);

            // Obtener los vectores y valores propios
            Eigen::Vector3f eigen_values = pca.getEigenValues();
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*object_cluster, centroid);

            // Ejemplo de log:
            ROS_INFO("Procesamiento completo");

            // Puedes publicar aquí la nube final o los resultados.
            // Publicar el clúster seleccionado
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*object_cluster, output);
            //output.header = cloud_msg->header;
            pub_.publish(output);

            // Mostrar resultados
            ROS_INFO_STREAM("Centroid: " << centroid.transpose());
            ROS_INFO_STREAM("Eigenvalues: " << eigen_values.transpose());
            ROS_INFO_STREAM("Eigenvectors (columns):\n" << eigen_vectors);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_segmentation");
    ros::NodeHandle nh;
    ObjectSegmentation obj_seg(nh);
    ros::Rate rate(100);  // Procesar a 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        obj_seg.processPointCloud();
        rate.sleep();
    }
}
