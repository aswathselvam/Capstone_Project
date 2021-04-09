#include <torch/torch.h>
#include <iostream>
using namespace std;

struct Net : torch::nn::Module {
    Net(int64_t N,int64_t M)
      : linear(register_module("linear", torch::nn::Linear(N,M))) {
          another_bias = register_parameter("b",torch::rand(M));
      }
    torch::Tensor forward(torch::Tensor input) {
        return linear(input)+another_bias;
    }
    torch::nn::Linear linear;
    torch::Tensor another_bias;
};

int main() {
    Net net(4,5);
    for(const auto &p:net.named_parameters()) {
        cout<<p.key()<<":"<<p.value()<<endl;
    }
    return 0;
}