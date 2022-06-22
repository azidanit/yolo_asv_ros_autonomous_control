#include <iostream>
#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort

using namespace std;

vector<pair<int, int> > sortArr(int arr[], int n)
{
  
    // Vector to store element
    // with respective present index
    vector<pair<int, int> > vp;
  
    // Inserting element in pair vector
    // to keep track of previous indexes
    for (int i = 0; i < n; ++i) {
        vp.push_back(make_pair(arr[i], i));
    }
  
    // Sorting pair vector
    sort(vp.begin(), vp.end());
  
    // Displaying sorted element
    // with previous indexes
    // corresponding to each element
    cout << "Element\t"
         << "index" << endl;
    for (int i = 0; i < vp.size(); i++) {
        cout << vp[i].first << "\t"
             << vp[i].second << endl;
    }

    return vp;
}

int main(){
    vector<pair<int, int> > sorter_arr;
    int arr[] = { 2, 5, 3, 7, 1 };
    int n = sizeof(arr) / sizeof(arr[0]);
    sorter_arr = sortArr(arr, n);

    std::cout << sorter_arr[0].first << " " << sorter_arr[0].second << "\n";
    return 0;
}