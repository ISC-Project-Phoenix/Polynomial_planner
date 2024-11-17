#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

class Polynomial {
    public:
    // TODO what does std::pmr::vec mean compared to std::vec ???
    Polynomial(std::pmr::vector<float>& vect) {
        this->vect = vect;
    }
    Polynomial() = default;
    // store the vector
    private:
        std::pmr::vector<float> vect;
    public:
    // returns the y value at a given X.
    float poly(float x) {
        float result = 0;
        for( int i = 0; i < vect.size(); i++ ) {
            int power = vect.size() - i;
            // TODO this pow function might be wrong
            result += vect[i] * pow(x, power); // a[n] * x ^ n
        }
        return result;
    }
    float polyDirvative(float x) {
        float result = 0;
        for( int i = 0; i < vect.size() -1; i++ ) {
            int power = vect.size() - i - 1;
            // todo finish?
            result += vect[i] * i * pow(x, power); // a[n] * n * x ^ (n - 1)
        }
    }
};
