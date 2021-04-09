#include <torch/torch.h>
#include <iostream>
using namespace std;

int main() {
    torch::Tensor mat = torch::ones({3,3});
    torch::Tensor one = torch::ones({3,3});
    cout << torch::add(mat,one) << endl;
    return 0;
}