#include <plan_env/voxblox_map.h>
#include <sstream>
#include <fstream>
#include <algorithm>

using namespace fast_planner;

voxblox::TsdfServer* tsdf_server;
// std::unique_ptr<voxblox::TsdfServer> tsdf_server;
Eigen::Vector3d map_ori_(-10.0, -15.5, 0.0);



Eigen::Vector3i map_voxel_num_;

string TSDF_Floder = "/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/tsdf_files/2";
string gt_TSDF_File = "/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/tsdf_files/gt.tsdf";

std::vector<string> tsdf_file_name_list;
std::vector<string> tsdf_file_name_list_without_folder;

void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos){
    for(int i = 0; i < 3; i++){
        pos[i] = id[i] * 0.2 + map_ori_[i];
        pos[i] += 0.2 / 2.0;
    }
}

void getFileNames(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + "/" + ptr->d_name);

            tsdf_file_name_list_without_folder.push_back(ptr->d_name);
        }
    }
    closedir(pDir);
}

SDFMap::VoxelStatusNum eval_map(const string& filename, voxblox::Layer<voxblox::TsdfVoxel>* tsdf_gt){
    int explored_voxel, unexplored_voxel, occupied_voxel, free_voxel;

    occupied_voxel = free_voxel = 0;
    unexplored_voxel = map_voxel_num_[0] * map_voxel_num_[1] * map_voxel_num_[2];

    std::cout << "size of map_voxel_num: " << map_voxel_num_.transpose() << std::endl;

    for(int x = 0; x < map_voxel_num_[0]; x++)
        for(int y = 0; y < map_voxel_num_[1]; y++)
            for(int z = 0; z < map_voxel_num_[2]; z++){
                Eigen::Vector3d pos;
                Eigen::Vector3i id(x, y, z);
                indexToPos(id, pos);
                voxblox::Point pt(pos[0], pos[1], pos[2]);

                const voxblox::TsdfVoxel* voxel_cur_;
                voxel_cur_ = tsdf_server->getTsdfMapPtr()->getTsdfLayer().getVoxelPtrByCoordinates(pt);
                
                if(voxel_cur_ != nullptr){
                    if(voxel_cur_->distance < 0.2){
                        occupied_voxel++;
                    }
                    else{
                        free_voxel++;
                    }
                }
            }

    explored_voxel = occupied_voxel + free_voxel;
    unexplored_voxel -= explored_voxel;

    double tsdf_rmse;
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_cur = tsdf_server->getTsdfMapPtr()->getTsdfLayerPtr();

    tsdf_rmse = voxblox::utils::evaluateLayersRmse(*tsdf_cur, *tsdf_gt);

    SDFMap::VoxelStatusNum res(explored_voxel, unexplored_voxel, occupied_voxel, free_voxel );
    res.tsdf_rmse = tsdf_rmse;

    return res;
}

void saveToFile(std::vector<SDFMap::VoxelStatusNum>& voxel_status_num_list){
    string file_name = TSDF_Floder + "/voxel_status_num.txt";

    ofstream file_output;

    for(int i = 0; i < voxel_status_num_list.size(); i++){
        file_output.open(file_name, ios::out | ios::app);

        file_output << tsdf_file_name_list_without_folder[i].substr(0, 5) << "\t"
                    << voxel_status_num_list[i].explored_voxel << "\t" 
                    << voxel_status_num_list[i].free_voxel << "\t"
                    << voxel_status_num_list[i].occupied_voxel << "\t"
                    << voxel_status_num_list[i].unexplored_voxel << "\t"
                    << voxel_status_num_list[i].tsdf_rmse << std::endl;

        std::cout << tsdf_file_name_list_without_folder[i].substr(0, 5) << "\t"
                    << voxel_status_num_list[i].explored_voxel << "\t" 
                    << voxel_status_num_list[i].free_voxel << "\t"
                    << voxel_status_num_list[i].occupied_voxel << "\t"
                    << voxel_status_num_list[i].unexplored_voxel << "\t"
                    << voxel_status_num_list[i].tsdf_rmse << std::endl;

        file_output.close();
    }
}

bool cmp(string& s1, string& s2){
    return s1 < s2;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "eval_voxblox_map_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    // tsdf_server.reset(new voxblox::TsdfServer(nh, nh_private));
    tsdf_server = new voxblox::TsdfServer(nh, nh_private);

voxblox::TsdfServer* tsdf_gt_server;
tsdf_gt_server = new voxblox::TsdfServer(nh, nh_private);
    // string TSDF_Floder = "/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/esdf_files/2";

    
    std::cout << "start getting all tsdf files" << std::endl;
    getFileNames(TSDF_Floder, tsdf_file_name_list);
    std::cout << "finish getting all tsdf files" << std::endl;
    sort(tsdf_file_name_list.begin(), tsdf_file_name_list.end(), cmp);
    sort(tsdf_file_name_list_without_folder.begin(), tsdf_file_name_list_without_folder.end(), cmp);

    std::vector<SDFMap::VoxelStatusNum> voxel_status_num_list;
    voxel_status_num_list.resize(tsdf_file_name_list.size());

    map_voxel_num_[0] = (int)(20 / 0.2);
    map_voxel_num_[1] = (int)(20 / 0.2);
    map_voxel_num_[2] = (int)(2 / 0.2);

    tsdf_gt_server->loadMap(gt_TSDF_File);
    // voxblox::Layer<voxblox::TsdfVoxel> tsdf_gt = voxblox::Layer<voxblox::TsdfVoxel>(*tsdf_server->getTsdfMapPtr()->getTsdfLayerPtr());
    // tsdf_server.release();
    voxblox::Layer<voxblox::TsdfVoxel> *tsdf_gt_ptr = tsdf_gt_server->getTsdfMapPtr()->getTsdfLayerPtr();
    // voxblox::Layer<voxblox::TsdfVoxel> tsdf_gt = *(tsdf_gt_ptr);

    // delete tsdf_server;

    // tsdf_server.reset(new voxblox::TsdfServer(nh, nh_private));


    for(int i = 0; i < tsdf_file_name_list.size(); i++){
            tsdf_server = new voxblox::TsdfServer(nh, nh_private);
        if(tsdf_server->loadMap(tsdf_file_name_list[i])){
            std::cout << "succeed loading map" <<std::endl;
        }
        else{
            std::cout << "fail loading map" <<std::endl;
        }

        voxel_status_num_list[i] = eval_map(tsdf_file_name_list[i], tsdf_gt_ptr);

        cout << tsdf_file_name_list[i] << endl;
        delete tsdf_server;
    }

    std::cout << "start saving to file" << std::endl;
    saveToFile(voxel_status_num_list);
    std::cout << "finish saving to file" << std::endl;

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}