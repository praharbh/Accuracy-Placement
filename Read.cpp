#include "Read.h"

// Function to read the path from a csv file
vector<math::Transform> Read::CSV(string path)
{
    vector<math::Transform> frames = vector<math::Transform>();
    math::Transform frame = math::Transform();
    math::Matrix33 matrix;
    
    ifstream file(path);

    if (!file.is_open()) throw std::runtime_error("Could not open file");

    vector<double> row = vector<double>();
    string line = "";
    string word = "";

    double val;
    while (getline(file, line))
    {
        row.clear();

        stringstream ss(line);

        while (ss >> val) {

            row.push_back(val);

            if (ss.peek() == ',') ss.ignore();

        }
   
        if (row.size() == 3) {
            matrix(0, 0) = -1; matrix(0, 1) = 0; matrix(0, 2) = 0;
            matrix(1, 0) = 0; matrix(1, 1) = 1; matrix(1, 2) = 0;
            matrix(2, 0) = 0; matrix(2, 1) = 0; matrix(2, 2) = -1;
            frame = math::Transform(matrix);
            frame.translation() << row[0] / 1000, row[1] / 1000, row[2] / 1000;
            frames.push_back(frame);
        } 
        /*else if (row.size() == 12) {
            matrix(0, 0) = row[3]; matrix(0, 1) = row[6]; matrix(0, 2) = row[9];
            matrix(1, 0) = row[4]; matrix(1, 1) = row[7]; matrix(1, 2) = row[10];
            matrix(2, 0) = row[5]; matrix(2, 1) = row[8]; matrix(2, 2) = row[11];
            frame = math::Transform(matrix);
            frame.translation() << row[0] / 1000, row[1] / 1000, row[2] / 1000;
            frames.push_back(frame);
        } */
    }
    return frames;
}

vector<math::Transform> Read::Relocate(math::Transform location, 
    vector<math::Transform> frames)
{
    vector<math::Transform> reFrames;
    for (int i = 0; i < frames.size(); i++) {
        reFrames.push_back(location * frames[i]);
    }
    return reFrames;
}
