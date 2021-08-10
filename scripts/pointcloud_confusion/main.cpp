#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <string>
#include <map>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

class ColorLabel{
public:
    std::string color_;
    int label_;
    std::string name_;

    ColorLabel(std::string color, int label, std::string name){
        color_ = color;
        label_ = label;
        name_=name;
    }
};

class ClassScore{
public:

    double precision_;
    double recall_;
    double f1score_;

    std::string name_;

    ClassScore(std::string& name, double precision, double recall, double f1score) :
    name_(name), precision_(precision), recall_(recall), f1score_(f1score){};
};

std::map<std::string, ColorLabel> read_labels(std::string& label_path){

    std::map<std::string, ColorLabel> colorlabels;

    std::ifstream  data(label_path);
    std::string line;
    std::vector<std::vector<std::string> > parsedCsv;
    while(std::getline(data,line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<std::string> parsedRow;
        while(std::getline(lineStream,cell,','))
        {
            parsedRow.push_back(cell);
        }
        try{
            std::string name = parsedRow[0];
            std::string red = parsedRow[1];
            std::string green = parsedRow[2];
            std::string blue = parsedRow[3];
            int id = stoi(parsedRow[5]);
            std::string color = red + green + blue;

            colorlabels.insert(std::pair<std::string, ColorLabel>(color, ColorLabel(color, id, name)));

        }catch (const std::invalid_argument& ia) {
            std::cerr << "reading headers: \n" << line << '\n';
        }



    }

    return colorlabels;
}

std::string find_name_by_label(int label, std::map<std::string, ColorLabel> colorLabels){

    for (auto const& colorlabel : colorLabels)
    {
        if (label == colorlabel.second.label_){
            return colorlabel.second.name_;
        }
    }
    return "NotFound";
}

int main() {

    std::string base_path = "";
    std::string gt_path = base_path + "gt_semantic_mesh.ply";
    std::string src_path = base_path + "gt_pose_maskrcnn.ply";
    std::string labels_path = base_path + "sunrgbd_13_segmentation_mapping.csv";
    std::string confusion_m_path = base_path + "confusion_matrix.csv";

    std::map<std::string, ColorLabel> colorLabels = read_labels(labels_path);
    int label_size = colorLabels.size();
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> confusion_matrix =
                            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Constant(label_size, label_size, 0);


    PointCloudPtr src_cloud (new PointCloud);
    PointCloudPtr gt_cloud (new PointCloud);

    pcl::PLYReader Reader;
    Reader.read(src_path, *src_cloud);
    Reader.read(gt_path, *gt_cloud);

    std::cout << "Loaded "
              << src_cloud->width * src_cloud->height
              << " data points from the source cloud and \n"
              << gt_cloud->width * gt_cloud->height
              << " data points from the target cloud"<<std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    tree_->setInputCloud(gt_cloud);

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    int counter = 0;
    int percentage_step = src_cloud->points.size () / 100;
    int percentage = 0;

    for(int nIndex = 0; nIndex < src_cloud->points.size (); nIndex++)
    {
        Point src = src_cloud->points[nIndex];
        std::string src_color = std::to_string(src.r) + std::to_string(src.g) + std::to_string(src.b);
        int src_label = colorLabels.find(src_color)->second.label_;

        tree_->nearestKSearch(src, 1, nn_indices, nn_dists);
        Point gt = gt_cloud->points[nn_indices[0]];
        std::string target_color = std::to_string(gt.r) + std::to_string(gt.g) + std::to_string(gt.b);
        int target_label = colorLabels.find(target_color)->second.label_;

        confusion_matrix(target_label, src_label) += 1;

        if (counter > percentage_step){
            counter = 0;
            percentage += 1;
            std::cout<<"processed: "<<percentage << "%" <<std::endl;
        }

        counter++;
    }

    auto true_label_sums = confusion_matrix.colwise().sum();
    auto predicted_label_sums = confusion_matrix.rowwise().sum();

    std::vector<ClassScore> scores;

    for (int label = 0; label < label_size; label++){

        double freq = double(confusion_matrix(label, label));
        double precision = freq / predicted_label_sums[label];
        double recall = freq / true_label_sums[label];
        double f1score = 2 * (precision * recall) / (precision + recall);
        std::string name = find_name_by_label(label, colorLabels);

        ClassScore class_score(name,  precision, recall,  f1score);
        scores.push_back(class_score);
        std::cout<<"name: "<<name<<"\tprecision: "<<precision<<"\trecall: "<<recall<<"\tf1score: "<<f1score<<std::endl;

    }

    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols,
                                           ", ", "\n");

    std::ofstream file(confusion_m_path);
    file << confusion_matrix.format(CSVFormat);
    file.close();

    return (0);
}
