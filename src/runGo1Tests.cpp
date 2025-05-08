#include "go1_cpp_cmake/go1TestFunctions.h"

/*
    As you can see, there will be more functions to test as things progress,
    and of course, break.
*/

int main(int argc, char ** argv) {
    go1TestFunctions test_proctor = go1TestFunctions();
    int test1 = test_proctor.testZeroPosErrorGRF();
    int test2 = test_proctor.testZeroPosErrorWalk();
    int test3 = test_proctor.testNonzeroPosErrorGRF();
    int test4 = test_proctor.testNonzeroPosErrorWalk();
    int test5 = test_proctor.testRaibertHeuristic();
    int test6 = test_proctor.testAmirHLIP();
    int test7 = test_proctor.testSwingPD();
    int test8 = test_proctor.testBezierPos();
    int test9 = test_proctor.testBezierVel();
    int test10 = test_proctor.testNumericJacobian();

    int numTests = 10;
    int numTestsPassed = test1 + test2 + test3 + test4 + test5 + test6 + test7 + test8 + test9 + test10;

    if (numTestsPassed == numTests) {
        std::cout << "\ntests passed: "<< numTestsPassed << "/" << numTests << "\nbeautiful work" << std::endl;
    } else {
        std::cout << "\ntests passed: "<< numTestsPassed << "/" << numTests << "\nfind the bugs, they're in my walls" << std::endl;
    }
    return 0;
}