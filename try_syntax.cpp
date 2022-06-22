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
    // vector<pair<int, int> > sorter_arr;
    // int arr[] = { 2, 5, 3, 7, 1 };
    // int n = sizeof(arr) / sizeof(arr[0]);
    // sorter_arr = sortArr(arr, n);

    // std::cout << sorter_arr[0].first << " " << sorter_arr[0].second << "\n";
    
    // std::cout << (2 < 5 && 5 < 8) << " TF " << (2 < 9 && 9 < 8) << "\n";
    double value_cur = 0;
    double value_before = 0;

    double alpha = 0.3;

    double val_out = 0;

    while(true){
        cin >> value_cur;
        if(value_cur == -9999)
            break;

        cout << "INPUT NEW VAL : " << value_cur << "\n";

        val_out = (alpha * value_cur) + (1-alpha) * value_before;
        cout << "OUT       VAL : " << val_out << "\n";

        value_before = val_out;


    }

    return 0;
}