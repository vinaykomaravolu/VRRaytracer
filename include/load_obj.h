#ifndef LOADOBJ_H
#define LOADOBJ_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <vector>

int load_obj(const char *path, std::vector<Eigen::Vector3d> &Indices)
{
    std::vector<Eigen::Vector3d> temp_vertices;
    std::vector<Eigen::Vector3d> temp_faces;
    std::vector<unsigned int> vertexIndices;
    FILE *file = fopen(path, "r");
    if (file == NULL)
    {
        printf("UNABLE TO READ FILE!\n");
        return false;
    }
    while (1)
    {

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break;
        if (strcmp(lineHeader, "v") == 0)
        {
            float x;
            float y;
            float z;
            fscanf(file, "%f %f %f\n", &x, &y, &z);
            temp_vertices.push_back(Eigen::Vector3d(double(x),double(y),double(z)));
        }
        else if (strcmp(lineHeader, "f") == 0)
        {
            std::string vertex1, vertex2, vertex3;
            unsigned int vertexIndex[3];
            int matches = fscanf(file, "%d %d %d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2]);
            if (matches != 3)
            {
                printf("File can't be read by our simple parser \n");
                return false;
            }
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
        }
    }

    for( unsigned int i=0; i<vertexIndices.size(); i++ ){
        unsigned int vertexIndex = vertexIndices[i];
        Eigen::Vector3d vertex = temp_vertices[vertexIndex - 1];
        Indices.push_back(vertex);
    }
}

#endif