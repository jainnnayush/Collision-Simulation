#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

using namespace std;

class HeapClass {
private:
    vector<double> heap;
    vector<int> position;
    vector<int> box_num;
    int length;

public:
    HeapClass(int num) {
        heap.resize(num, -1);
        position.resize(num, -1);
        box_num.resize(num, -1);
        length = 0;
    }

    bool heap_comparator(int idx1, int idx2) {
        if (heap[idx1] != heap[idx2]) {
            return heap[idx1] < heap[idx2];
        } else {
            return idx1 < idx2;
        }
    }

    void swap_index(int idx1, int idx2) {
        swap(heap[idx1], heap[idx2]);
        position[box_num[idx1]] = idx2;
        position[box_num[idx2]] = idx1;
        swap(box_num[idx1], box_num[idx2]);
    }

    void heapify(int curr) {
        int parent = (curr - 1) / 2;
        if (parent >= 0) {
            if (heap_comparator(curr, parent)) {
                swap_index(parent, curr);
                heapify(parent);
            }
        }
    }

    void heapify_down(int curr) {
        int left_child = 2 * curr + 1;
        int right_child = 2 * curr + 2;
        int smaller_child = left_child;
        if (left_child >= length) {
            return;
        } else if (right_child >= length) {
            smaller_child = left_child;
        } else {
            if (heap_comparator(right_child, left_child)) {
                smaller_child = right_child;
            }
        }
        if (heap_comparator(smaller_child, curr)) {
            swap_index(curr, smaller_child);
            heapify_down(smaller_child);
        }
    }

    void insert(int time, int box_number) {
        heap[length] = time;
        box_num[length] = box_number;
        position[box_number] = length;
        length++;
        heapify(length - 1);
    }

    void remove(int box_number) {
        int heap_index = position[box_number];
        swap_index(heap_index, length - 1);
        heap[length - 1] = -1;
        box_num[length - 1] = -1;
        position[box_number] = -1;
        length--;
        if (heap[heap_index] != -1) {
            heapify(heap_index);
            heapify_down(heap_index);
        }
    }

    pair<int, int> top() {
        return make_pair(heap[0], box_num[0]);
    }
};

const double max_float = pow(10, 20);

double time_for_collision(int idx, vector<double>& velocity, vector<double>& pos_last_col, vector<double>& time_last_col, double curr_time) {
    if (velocity[idx + 1] - velocity[idx] >= 0) {
        return max_float;
    }
    double x1 = pos_last_col[idx] + (curr_time - time_last_col[idx]) * velocity[idx];
    double x2 = pos_last_col[idx + 1] + (curr_time - time_last_col[idx + 1]) * velocity[idx + 1];
    return curr_time + (x2 - x1) / (velocity[idx] - velocity[idx + 1]);
}

double pos_for_collision(int idx, vector<double>& velocity, vector<double>& pos_last_col, vector<double>& time_last_col, double time_taken_for_col) {
    return pos_last_col[idx] + velocity[idx] * (time_taken_for_col - time_last_col[idx]);
}

pair<double, double> updated_velocity(int idx, vector<double>& mass, vector<double>& velocity) {
    double v1 = ((mass[idx] - mass[idx + 1]) * velocity[idx] + (2 * mass[idx + 1] * velocity[idx + 1])) / (mass[idx] + mass[idx + 1]);
    double v2 = ((2 * mass[idx] * velocity[idx] + (mass[idx + 1] - mass[idx]) * velocity[idx + 1])) / (mass[idx] + mass[idx + 1]);
    return make_pair(v1, v2);
}

vector<tuple<double, int, double>> listCollisions(vector<double>& M, vector<double>& x, vector<double>& v, vector<double>& m, double T) {
    int num_boxes = M.size();
    vector<double> time_last_col(num_boxes, 0);
    vector<double> pos_last_col = x;
    HeapClass time_heap(num_boxes - 1);

    for (int i = 0; i < num_boxes - 1; i++) {
        time_heap.insert(time_for_collision(i, v, pos_last_col, time_last_col, 0), i);
    }

    int counter = 0;
    vector<tuple<double, int, double>> ans;

    while (counter < T) {
        counter++;

        double time_taken_for_col;
        int left_box_idx;
        tie(time_taken_for_col, left_box_idx) = time_heap.top();

        if (time_taken_for_col > T) {
            break;
        }

        double position_of_col = pos_for_collision(left_box_idx, v, pos_last_col, time_last_col, time_taken_for_col);

        time_last_col[left_box_idx] = time_taken_for_col;
        time_last_col[left_box_idx + 1] = time_taken_for_col;
        pos_last_col[left_box_idx] = position_of_col;
        pos_last_col[left_box_idx + 1] = position_of_col;

        ans.push_back(make_tuple(time_taken_for_col, left_box_idx, position_of_col));

        double v1, v2;
        tie(v1, v2) = updated_velocity(left_box_idx, M, v);
        v[left_box_idx] = v1;
        v[left_box_idx + 1] = v2;

        time_heap.remove(left_box_idx);
        time_heap.insert(max_float, left_box_idx);

        if (left_box_idx - 1 >= 0) {
            time_heap.remove(left_box_idx - 1);
            time_heap.insert(time_for_collision(left_box_idx - 1, v, pos_last_col, time_last_col, time_taken_for_col), left_box_idx - 1);
        }

        if (left_box_idx + 2 < num_boxes) {
            time_heap.remove(left_box_idx + 1);
            time_heap.insert(time_for_collision(left_box_idx + 1, v, pos_last_col, time_last_col, time_taken_for_col), left_box_idx + 1);
        }
    }

    return ans;
}

int main() {
    vector<double> M = {1.0, 2.0, 3.0};
    vector<double> x = {0.0, 0.0, 0.0};
    vector<double> v = {1.0, 2.0, 3.0};
    vector<double> m = {1.0, 2.0, 3.0};
    double T = 10.0;

    vector<tuple<double, int, double>> collisions =
}