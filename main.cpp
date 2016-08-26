/*
 * LICENSE!!!
 */

#include <iostream>
#include <string>
#include <sys/stat.h>
#include <math.h>
#include <stdio.h>
#include <sys/types.h>
#include <ctime>
#include <utility>
#include <algorithm>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define NOMINMAX


using namespace cv;
using namespace boost;


#define SEP "/"


typedef std::pair<int, int> Pair;

struct Triple
{
    int k0;
    int k1;
    int k2;
};


struct Adaptive_Grid
{
    std::vector<Triple> Edges;
    std::vector<Triple> pos;
    string dir_conv;
};


struct Graph
{
    std::vector<Triple> Edges;
};


struct Data_Label
{
    string label;
    std::vector<int> data;
};


struct Labels
{
};


struct Pos2D3D
{
    std::vector<Triple> pos2D;
    std::vector<Triple> pos3D;
};

//typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
//typedef std::pair<int, int> Edge;


void image_graph(int imWidth,int imHeight,string crd, Mat image, int smin, int thresholding_m);
void image_graph_run();
void image_graph_calc(const string &crd, const string &dir_input, const string &file_input);
Adaptive_Grid image_graph_AMR_2D_Adaptive_grid(int imWidth,int imHeight,string crd,Mat im,string dir_conv,string dir_Edges,string dir_QuadTree,string dir_output);
Graph grid_grid_all(Mat im, string file_path,Adaptive_Grid Edges_pos_dir_conv,int dz);
void save0(string url,std::vector<Triple> Edges);
void save0(string url,std::vector<int> data);
void save0(string url,string label);
Data_Label graph_graph_all(Graph G,std::vector<Triple> pos);
Labels load0(string url);
void write0(string url,Labels labels);
void plot0(string url, int imHeight, int imWidth, std::vector<Triple> pos);
Labels load0(string url);
Graph load1(string url);
int number_of_nodes(Graph g);
Graph subgraph(Graph g,int n1, int n2);
Graph convert_node_labels_to_integers(Graph g);
//Pos2D3D image_graph_help_grid_pos2D(int n,std::vector<Triple> pos);


void image_graph(int imWidth,int imHeight,string crd, Mat image, int smin, int thresholding_m)
{
    // timing
    printf("image width = %d, image height = %d\n", imWidth, imHeight);

    if (smin<2)
        {
            printf(" The minimum block size can not be less than 2 pixels \n");
            return;
        }
    int D = log(min(imWidth, imHeight)/smin)+float(3/2);
    printf("The margins of the grid will be %d\n", D);
    int w = imWidth - (D*2);
    int h = imHeight - (D*2);
    printf("The Width of the grid = %d ,  and the Height of the grid = %d\n",w,h);
    int dmax = log(min(w, h)/smin) + float(3/2);
    printf("The maximum depth the grid could be divided into = %d\n", dmax);
    if (smin<=16) // set Guassian kernel
        {
            float v = 0.5*smin;
        }
    else
        {
            float v= 8;
        }

    return;
}


void image_graph_run()
{
    string dir_input("../data/images/");
    string file_name("Control1.tif");
    string gridtype("rectangular");
    image_graph_calc(gridtype, dir_input, file_name);
    return;
}


void image_graph_calc(const string& crd, const string& dir_input, const string& file_input)
{
    string name("Adaptive_grid_");
    string dir_output = dir_input+"Output_"+name+file_input+SEP;
    string subfolders[7] = {"data_posi","data_conv","data_grph","data_datn","data_prop","data_readable","plot_grid"};
    string file_path = dir_input+file_input;
    mkdir(dir_output.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    for (int i = 0; i < (sizeof(subfolders)/sizeof(*subfolders)); ++i)
        {
            string s1=dir_output+subfolders[i];
            mkdir(s1.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            //std::cout<<s1<<std::endl;
        }
    string s2=dir_output+"data_readable"+SEP+"data_readable.txt";
    remove(s2.c_str());

    std::cout<<"image_graph_AMR"<<std::endl;
    Mat im = imread(file_path.c_str(), IMREAD_GRAYSCALE);

    int imHeight = im.rows;
    int imWidth = im.cols;
    int AMR = 1;///to be removed
    string dir_conv = dir_output+"data_conv"+SEP+"data_conv";
    string dir_Edges = dir_output+"data_Edges"+SEP+"data_Edges";
    string dir_QuadTree = dir_output+"data_QuadTree"+SEP+"data_QuadTree";

    if(im.dims == 2)
        {
            printf("2D image_graph ..\n");
        }
    else
        {
            printf("3D image_graph ..\n"); // in next steps I will work on 3D version
        }
    Adaptive_Grid Edges_pos_dir_conv=image_graph_AMR_2D_Adaptive_grid(imWidth,imHeight,crd,im,dir_conv,dir_Edges,dir_QuadTree,dir_output);
    std::cout<<"graph"<<std::endl;
    time_t temp1 = time(0);
    Graph graph = grid_grid_all(im,file_path,Edges_pos_dir_conv,1);
    save0(dir_output+SEP+"data_grph"+SEP+"data_grph.npy",graph.Edges);
    std::cout<<temp1<<"the time consumed to build the graph is "<<(time(0)-temp1)<<std::endl;
    std::cout<< "obs network"<<std::endl;

    time_t temp2 = time(0);
    Data_Label data_label = graph_graph_all(graph,Edges_pos_dir_conv.pos);
    std::cout<<temp1<<"the time consumed to measure the graph's properties quantitatively is "<<(time(0)-temp2)<<std::endl;
    save0(dir_output+SEP+"data_datn"+SEP+"data_datn.npy",data_label.data);
    save0(dir_output+SEP+"data_prop"+SEP+"data_prop.npy",data_label.label);

    Labels labels=load0(dir_output+SEP+"data_prop"+SEP+"data_prop.npy");

    write0(dir_output+SEP+"data_readable"+SEP+"data_readable.txt",labels);
    // with open(dir_output"+SEP+"data_readable"+SEP+"data_readable.txt","a") as out:
    //     out.write('\t'.join([str(a) for a in numpy.hstack(labels)]))
    //     out.write('\n')
    //     datn=numpy.load(os.path.join(dir_output,'data_datn','data_datn.npy'))
    //     out.write('\t'.join([str(a) for a in numpy.hstack(datn)]))
    //     out.write('\n')

    plot0(dir_output+SEP+"data_grph"+SEP+"data_grph.npy",imHeight,imWidth,Edges_pos_dir_conv.pos);
}


Adaptive_Grid image_graph_AMR_2D_Adaptive_grid(int imWidth,int imHeight,string crd,Mat im,string dir_conv,string dir_Edges,string dir_QuadTree,string dir_output)
{
    Adaptive_Grid ag;
    return ag;
}


Graph grid_grid_all(Mat im, string file_path, Adaptive_Grid Edges_pos_dir_conv,int dz)
{
    Graph graph;
    return graph;
}


Data_Label graph_graph_all(Graph G,std::vector<Triple> pos)
{
    Data_Label data_label;
    return data_label;
}


void plot0(string url, int imHeight, int imWidth, std::vector<Triple> pos)
{
    int ly = imHeight;
    int lx = imWidth;
    Graph gn=load1(url);
    int N=number_of_nodes(gn);
    //Return a copy of the graph gn with the nodes relabeled using consecutive integers.
    Graph gc=convert_node_labels_to_integers(subgraph(gn,N-N/1,N));
    //posi=load(dir_output+SEP+"data_posi"+SEP+"data_posi.npy").flatten()[0]
//    Pos2D3D pos2Dpos3D=image_graph_help_grid_pos2D(1,pos);
    // en=numpy.array([d['capa'] for u,v,d in gn.edges_iter(data=1)])
    // en=en/en.max()
    // ec=numpy.array([d['capa'] for u,v,d in gc.edges_iter(data=1)])
    // ec=ec/en.max()
    // matplotlib.pyplot.clf()
}


//Pos2D3D image_graph_help_grid_pos2D(int n,std::vector<Triple> pos)
//{
//}


void write0(string url,Labels labels)
{
}


void save0(string url,std::vector<Triple> Edges)
{
}


void save0(string url,std::vector<int> data)
{
}


void save0(string url,string label)
{
}


Labels load0(string url)
{
    Labels labels;
    return labels;
}


Graph load1(string url)
{
    Graph g;
    return g;
}


int number_of_nodes(Graph g)
{
    int n;
    return n;
}


Graph subgraph(Graph g,int n1, int n2)
{
    Graph g1;
    return g1;
}


Graph convert_node_labels_to_integers(Graph g)
{
    Graph g1;
    return g1;
}


Pos2D3D convert_node_labels_to_integers(int nz, std::vector<Triple> pos)
{
    Pos2D3D p;
    return p;
}


int main( int argc, char** argv )
{
    image_graph_run();
    return 0;
}


// int main1( int argc, char** argv )
// {
//     int smin = 2; // user-defined parameter (Minimum block size) - thresholding works on 2px and more.
//     int thresholding_m = 2; // applied thresholding method (1: , 2:, 3: )
//     string inputImage("Cytoskeletal.png");
//     string gridtype("rectangular");
//     string image_dir("/home/next/Desktop/C/UpWork/PeterBrams/image_graph/images/");
//     string nwfolderName = "image_graph"+inputImage;
//     string imageName1 = image_dir+SEP+inputImage;
//     string imageName = imageName1.c_str();
//     string crd = "rectangular";
//     //string position_dir = nwfolderName+SEP+'data_posi';
//     if( argc > 1)
//     {
//         imageName = argv[1];
//     }
//     Mat image;
//     image = imread(imageName.c_str(), IMREAD_GRAYSCALE); // Read the file
//     if( image.empty() )                              // Check for invalid input
//     {
//         printf("Could not open or find the image\n") ;
//         return -1;
//     }

//     namedWindow( "Display window", WINDOW_AUTOSIZE ); // or WINDOW_KEEPRATIO [ Create a window for display].
//     imshow( "Display window", image );                // Show our image inside it.
//     waitKey(0);//  Wait for a keystroke in the window

//     const int dir_err = mkdir(nwfolderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//     // if (-1 == dir_err)
//     // {
//     //     printf("Error creating directory!\n");
//     //     exit(1);
//     // }
//     // To do: add many directories to save the results..
//     //------------------------------------------------//

//     if(image.dims == 2)
//     {
//       printf("2D image_graph ..\n");
//     }
//     else
//     {
//       printf("3D image_graph ..\n"); // in next steps I will work on 3D version
//     }

//     int imHeight = image.rows;
//     int imWidth = image.cols;
//     int AMR = 1;///to be removed
//     image_graph(imWidth, imHeight, crd, image, smin, thresholding_m);

//     return 0;
// }
