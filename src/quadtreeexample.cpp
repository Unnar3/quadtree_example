#include <ros/ros.h>
#include <Eigen/Dense>
#include <quad_tree_decimation/quad_tree/quad_tree.h>
#include <iostream>
#include <fstream>
using namespace std;


using namespace QTD;


void saveTriangles(std::vector<std::vector<int>> vert, std::vector<quadPoint> points){
    ofstream myfile;
    myfile.open ("/home/unnar/catkin_ws/src/quadtree_example/data/quadTreeTest/triangles.txt");

    myfile << "# Points" << std::endl;
    for(auto p : points){
        myfile << std::to_string(p.x) << "," << std::to_string(p.y) << std::endl;
    }
    myfile << "# Vertices" << std::endl;
    for(auto v : vert){
        myfile << std::to_string(v[0]) << "," << std::to_string(v[1]) << "," << std::to_string(v[2]) << std::endl;
    }

    myfile.close();
}

void saveQuad(std::vector<QuadTree::Cell> cells, string name){
    ofstream myfile;
    myfile.open ("/home/unnar/catkin_ws/src/quadtree_example/data/quadTreeTest/" + name);
    for(auto cell : cells){
        myfile << std::to_string(cell.x) << ",";
        myfile << std::to_string(cell.y) << ",";
        myfile << std::to_string(cell.width) << ",";
        myfile << std::to_string(cell.r) << ",";
        myfile << std::to_string(cell.g) << ",";
        myfile << std::to_string(cell.b) << ",";
        myfile << std::to_string(cell.cellType) << std::endl;
    }
    myfile.close();
}

void savePolygon(std::vector<QTD::quadPoint> qtd_boundary, std::vector<int> polygonstartIdx){
    ofstream myfile;
    myfile.open ("/home/unnar/catkin_ws/src/quadtree_example/data/quadTreeTest/polygon.txt");
    for(auto n : polygonstartIdx){
        myfile << std::to_string(n);
        std::cout << n << std::endl;
        if(n != polygonstartIdx.back()){
            myfile << ",";
        } else {
            myfile << std::endl;
        }
    }

    for(auto p : qtd_boundary){
        myfile << std::to_string(p.x) << "," << std::to_string(p.y) << std::endl;
    }
}


int main(int argc, char **argv) {

    // ros::init(argc, argv, "quadTreeExample");

    // ros::Rate loop_rate(10);

    QTD::QuadTree quad;
    quad.setMaxWidth(0.1);

    std::vector<QuadTree::Cell> cells0;
    quad.extractCells(cells0);
    saveQuad(cells0, "quad0.txt");

    std::vector<QTD::quadPoint> qtd_boundary;

    std::vector<int> polygonstartIdx;
    polygonstartIdx.push_back(qtd_boundary.size());

    qtd_boundary.push_back(quadPoint(2,2));
    qtd_boundary.push_back(quadPoint(6,1));
    qtd_boundary.push_back(quadPoint(9,2));
    qtd_boundary.push_back(quadPoint(9,6));
    qtd_boundary.push_back(quadPoint(8,9));
    qtd_boundary.push_back(quadPoint(4,9));
    qtd_boundary.push_back(quadPoint(4,7));
    qtd_boundary.push_back(quadPoint(2,6));
    qtd_boundary.push_back(quadPoint(1,4));
    qtd_boundary.push_back(quadPoint(2,2));

    polygonstartIdx.push_back(qtd_boundary.size());
    qtd_boundary.push_back(quadPoint(5,4));
    qtd_boundary.push_back(quadPoint(6,3));
    qtd_boundary.push_back(quadPoint(7,4));
    qtd_boundary.push_back(quadPoint(7,7));
    qtd_boundary.push_back(quadPoint(6,7));
    qtd_boundary.push_back(quadPoint(5,4));

    auto startNew = polygonstartIdx.begin();
    startNew++;
    for(size_t i = 0; i < qtd_boundary.size()-1; ++i){
        if(i+1 != *startNew)
            quad.insertSegment( qtd_boundary[i], qtd_boundary[i+1] );
        else{
            if(*startNew != polygonstartIdx.back())
                startNew++;
        }
    }

    std::vector<QuadTree::Cell> cells1;
    quad.extractCells(cells1);

    saveQuad(cells1, "quad1.txt");
    savePolygon(qtd_boundary, polygonstartIdx);

    polygonstartIdx.push_back(qtd_boundary.size());
    quad.markAsExternal(qtd_boundary, polygonstartIdx, quadPoint(1,1), quadPoint(9,9));
    std::vector<QuadTree::Cell> cells2;
    quad.extractCells(cells2);

    saveQuad(cells2, "quad2.txt");


    std::vector<std::vector<int>> vert;
    std::vector<quadPoint> points;
    quad.extractTriangles(points, vert);
    saveTriangles(vert, points);



    return 0;
}
