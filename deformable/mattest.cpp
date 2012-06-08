#include <Eigen/Core>
#include <Eigen/LU>
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

using namespace Eigen;

bool loadMatrix(const char *fname, MatrixXd& m)
{
    std::ifstream ifs(fname);
    if (!ifs.is_open()){
        std::cout << "failed to open(" << fname << ")" << std::endl;
        return false;
    }
    unsigned int r, c;
    ifs >> c >> r;
    m.resize(c,r);
    for (unsigned int i=0; i<c; i++){
        for (unsigned int j=0; j<r; j++){
            ifs >> m(i,j);
        }
    }
    return true;
}

bool loadGzMatrix(const char *fname, MatrixXd& m)
{
    std::ifstream ifs(fname);
    if (!ifs.is_open()){
        std::cout << "failed to open(" << fname << ")" << std::endl;
        return false;
    }

    boost::iostreams::filtering_streambuf<boost::iostreams::input> filter;
    filter.push(boost::iostreams::gzip_decompressor());
    filter.push(ifs);
    std::istream stream(&filter);

    unsigned int r, c;
    stream >> c >> r;
    char buf[10];
    stream.getline(buf, 10); // to read newline
    m.resize(c,r);
    for (unsigned int j=0; j<r; j++){
        double col[c];
        stream.read((char *)col, c*sizeof(double));
        for (unsigned int i=0; i<c; i++){
            m(i,j) = col[i];
        }
    }
    return true;
}


void saveMatrix(const char *fname, const MatrixXd& m)
{
    std::ofstream ofs(fname);
    ofs << m.cols() << " "  << m.rows() << std::endl; 
    for (int i=0; i<m.cols(); i++){
        for (int j=0; j<m.rows(); j++){
            ofs << m(i,j) << " ";
        }
        ofs << std::endl;
    }
}

void saveGzMatrix(const char *fname, const MatrixXd& m)
{
    std::ofstream ofs(fname);

    boost::iostreams::filtering_streambuf<boost::iostreams::output> filter;
    filter.push(boost::iostreams::gzip_compressor());
    filter.push(ofs);
    std::ostream stream(&filter);

    stream << m.cols() << " "  << m.rows() << std::endl; 
    stream.write((const char *)m.data(), m.cols()*m.rows()*sizeof(double));
}


int main(int argc, char *argv[])
{
    MatrixXd m;
#if 1
    loadGzMatrix("/tmp/test.mat", m);
#else
    m.resize(2,2);
    m(0,0)=0; m(0,1)=1;
    m(1,0)=2; m(1,1)=3;
#endif
    std::cout << m << std::endl;
    std::cout << m.data()[0] << "," << m.data()[1] << ","
              << m.data()[2] << "," << m.data()[3] << std::endl;
    saveGzMatrix("/tmp/test.mat2", m);
}
