#include "Localizer.h"

void read(vector<pair<string, PoseData>>& data)
{
    data.clear();
    ifstream fin("../data/pose.bin", ios::binary);
    while (true)
    {
        if (fin.peek() == EOF)
            break;
        
        PoseData temp;
        fin.read((char*)&temp, sizeof(temp));
        static int cloud_index = 0;
        string filename = "../data/" + to_string(cloud_index) + ".pcd";
        data.push_back(make_pair(filename, temp));
        cloud_index++;
    }
    fin.close();
    std::cout << "data size: " << data.size() << std::endl;
}

int main()
{
    // read data and map param
    vector<pair<string, PoseData>> vData;
    read(vData);
 
    // localizing 
    Localizer localizer;
    for (size_t i = 0; i < vData.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(vData[i].first, *cloud) == -1) 
        {
            PCL_ERROR("Couldn't read file rabbit.pcd\n");
            return(-1);
        }
      
        localizer.update(cloud, vData[i].second);
    }

}