#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <chrono>
using namespace std;

/* === Global Variables === */
int ROWS, COLS;
vector<vector<vector<int>>> DIST;  // precomputed distances for heuristic
vector<int> DIR;

inline char dir_to_char(int dir){
    return (dir == -COLS) ? 'W' : (dir == -1) ? 'A' : (dir == COLS) ? 'S' : 'D';
}

/* === Position Structure === */
typedef int Position;

inline Position pos_up(Position pos){
    return pos - COLS;
}

inline Position pos_down(Position pos){
    return pos + COLS;
}

inline Position pos_left(Position pos){
    return pos - 1;
}

inline Position pos_right(Position pos){
    return pos + 1;
}

const int MAX_CELLS = 256;
typedef bitset<MAX_CELLS> Bitset;
Bitset TARGETS, WALLS, FRAGILE_TILES;

inline int ptoi(const pair<int, int>& pos){
    return pos.first * COLS + pos.second;
}

inline pair<int, int> itop(int idx){
    return {idx / COLS, idx % COLS};
}

/* === State Structure === */
struct State{
    Position player;
    Bitset boxes;
    struct action{
        Position from;
        int dir = 0;
    } action;
    int idx;
    int parent_idx = -1;

    bool operator==(const State& other)const{
        return player == other.player && boxes == other.boxes;
    }

    /* For A star */
    int h_cost = 0;
    int g_cost = 0;

    void compute_heuristic(){
        /* Sum of all the distances from each box to the closest target */
        for (int b = boxes._Find_first(), i = 0; b < boxes.size(); b = boxes._Find_next(b), i++){
            int min_dist = INT_MAX, min_target = -1;
            auto [br, bc] = itop(b);
            for (int t = TARGETS._Find_first(), j = 0; t < TARGETS.size(); t = TARGETS._Find_next(t), j++){
                auto [tr, tc] = itop(t);
                int dist = DIST[j][br][bc];
                if (dist > min_dist) continue;
                min_dist = dist;
                min_target = t;
            }
            if (min_dist == INT_MAX){
                h_cost = INT_MAX;
                return;
            }
            h_cost += min_dist;
        }
    }

    /* Game mechanics */
    bool is_solved()const{
        return boxes == TARGETS;
    }

    bool is_dead()const{
        for (int b = boxes._Find_first(); b < boxes.size(); b = boxes._Find_next(b)){
            if (TARGETS.test(b)) continue; // on target

            bool Up = WALLS.test(pos_up(b)) || boxes.test(pos_up(b));
            bool Down = WALLS.test(pos_down(b)) || boxes.test(pos_down(b));
            bool Left = WALLS.test(pos_left(b)) || boxes.test(pos_left(b));
            bool Right = WALLS.test(pos_right(b)) || boxes.test(pos_right(b));
            bool UpLeft = WALLS.test(pos_up(pos_left(b))) || boxes.test(pos_up(pos_left(b)));
            bool UpRight = WALLS.test(pos_up(pos_right(b))) || boxes.test(pos_up(pos_right(b)));
            bool DownLeft = WALLS.test(pos_down(pos_left(b))) || boxes.test(pos_down(pos_left(b)));
            bool DownRight = WALLS.test(pos_down(pos_right(b))) || boxes.test(pos_down(pos_right(b)));

            /* If walls and boxes form a 2x2 square, it is a deadlock */
            if ((Up && Left && UpLeft) || (Up && Right && UpRight)) return true;
            if ((Down && Left && DownLeft) || (Down && Right && DownRight)) return true;
        }
        return false;
    }

    /* Hash */
    struct Hash{
        size_t operator()(const State& s)const{
            size_t seed = 0;
            boost::hash_combine(seed, s.player);
            const uint64_t* raw = (const uint64_t*)&s.boxes;
            for (size_t i = 0; i < sizeof(Bitset)/sizeof(uint64_t); i++)
                boost::hash_combine(seed, raw[i]);
            return seed;
        }
    };
};

vector<State> NODES;

struct Compare{
    bool operator()(const int& a, const int& b)const{
        const State& state_a = NODES[a];
        const State& state_b = NODES[b];
        return (state_a.g_cost + state_a.h_cost) > (state_b.g_cost + state_b.h_cost);
    }
};

/* === Helper Functions === */
bool is_free(Position pos, const State& state, bool isBox = false, bool considerBoxes = true){
    if (WALLS.test(pos)) return false;
    if (isBox && FRAGILE_TILES.test(pos)) return false;
    if (!considerBoxes) return true;
    if (state.boxes.test(pos)) return false;
    return true;
}

void detect_simple_deadlocks(){
    /* All reachable positions for boxes from target */
    unordered_set<Position> reachable;

    for (int t = TARGETS._Find_first(); t < TARGETS.size(); t = TARGETS._Find_next(t)){
        queue<Position> q;
        unordered_set<Position> visited;
        q.push(t);
        visited.insert(t);
        while (!q.empty()){
            Position curr = q.front(); q.pop();
            reachable.insert(curr);
            for (auto delta : DIR){
                Position newPos = curr + delta;
                Position playerPos = newPos + delta;
                if (visited.count(newPos)) continue;
                if (!is_free(newPos, State(), true, false)) continue;  // box can not be moved to position 'newPos'
                if (!is_free(playerPos, State(), false, false)) continue;  // player can not stand on position 'playerPos'
                visited.insert(newPos);
                q.push(newPos);
            }
        }
    }

    /* Mark all non-reachable positions as fragile tile */
    for (int pos = 0; pos < ROWS * COLS; pos++){
        if (WALLS.test(pos) || TARGETS.test(pos)) continue;
        if (reachable.count(pos)) continue;
        FRAGILE_TILES.set(pos);
    }
}

void compute_dist(){
    int target_size = TARGETS.count();
    DIST.resize(target_size, vector<vector<int>>(ROWS, vector<int>(COLS, INT_MAX)));
    for (int t = TARGETS._Find_first(), i = 0; t < TARGETS.size(); t = TARGETS._Find_next(t), i++){
        queue<Position> q;;
        q.push(t);
        auto [tr, tc] = itop(t);
        DIST[i][tr][tc] = 0;
        while (!q.empty()){
            Position curr = q.front(); q.pop();
            auto [r, c] = itop(curr);
            for (auto delta : DIR){
                Position newPos = curr + delta;
                auto [nr, nc] = itop(newPos);
                if (!is_free(newPos, State(), true)) continue;
                if (DIST[i][nr][nc] < DIST[i][r][c] + 1) continue;
                DIST[i][nr][nc] = DIST[i][r][c] + 1;
                q.push(newPos);
            }
        }
    }
    for (int i = 0; i < target_size; i++){
        for (int r = 0; r < ROWS; r++){
            for (int c = 0; c < COLS; c++){
                if (DIST[i][r][c] == INT_MAX) continue;
                DIST[i][r][c] *= DIST[i][r][c];
            }
        }
    }
}

State loadState(const string& filename){
    ifstream fin(filename);

    string line;
    vector<string> map;
    int r = 0;
    while (getline(fin, line)){
        map.push_back(line);
        r++;
    }
    fin.close();

    ROWS = map.size();
    COLS = map[0].size();
    DIR.push_back(-COLS);  // W
    DIR.push_back(-1);     // A
    DIR.push_back(COLS);   // S
    DIR.push_back(1);      // D

    State init;
    for (int pos = 0; pos < ROWS * COLS; pos++){
        auto [r, c] = itop(pos);
        if (map[r][c] == ' ') continue;
        else if (map[r][c] == '#') WALLS.set(pos);
        else if (map[r][c] == '@') FRAGILE_TILES.set(pos);
        else if (map[r][c] == '!'){
            init.player = pos;
            FRAGILE_TILES.set(pos);
        }
        else if (map[r][c] == '.') TARGETS.set(pos);
        else if (map[r][c] == 'o') init.player = pos;
        else if (map[r][c] == 'O'){
            init.player = pos;
            TARGETS.set(pos);
        }
        else if (map[r][c] == 'x') init.boxes.set(pos);
        else if (map[r][c] == 'X'){
            init.boxes.set(pos);
            TARGETS.set(pos);
        }
    }

    init.idx = NODES.size();
    NODES.push_back(init);

    detect_simple_deadlocks();
    compute_dist();

    return init;
}

Bitset reachable_positions(const State& state){
    Bitset reachable;
    queue<Position> q;
    q.push(state.player);
    reachable.set(state.player);

    while (!q.empty()){
        auto curr = q.front(); q.pop();
        for (auto delta : DIR){
            Position newPos = curr + delta;
            if (!is_free(newPos, state)) continue;
            if (reachable.test(newPos)) continue;
            reachable.set(newPos);
            q.push(newPos);
        }
    }
    return reachable;
}

string get_path(Position to, const State& state){
    queue<Position> q;
    unordered_map<Position, pair<Position, int>> parent;
    unordered_set<Position> visited;
    
    q.push(state.player);
    visited.insert(state.player);

    while (!q.empty()){
        Position curr = q.front(); q.pop();
        if (curr == to) break;

        for (auto delta : DIR){
            Position newPos = curr + delta;
            if (!is_free(newPos, state)) continue;
            if (visited.count(newPos)) continue;
            visited.insert(newPos);
            parent[newPos] = {curr, delta};
            q.push(newPos);
        }
    }

    /* Reconstruct path */
    string path;
    Position curr = to;
    while (!(curr == state.player)){
        auto p = parent[curr];
        path += dir_to_char(p.second);
        curr = p.first;
    }
    reverse(path.begin(), path.end());
    return path;
}

void print_path(const State& state){
    string full_path;
    int curr_idx = state.idx;
    while (curr_idx != -1){
        const State& curr = NODES[curr_idx];
        if (curr.parent_idx != -1){
            const State& parent = NODES[curr.parent_idx];
            full_path = get_path(curr.action.from, parent) + dir_to_char(curr.action.dir) + full_path;
        }
        curr_idx = curr.parent_idx;
    }
    cout << full_path << "\n";
}

/* === Solvers === */
void astar_solver(State& init){
    priority_queue<int, vector<int>, Compare> pq;
    unordered_set<State, State::Hash> visited;

    init.compute_heuristic();
    pq.push(init.idx);
    visited.insert(init);

    while (!pq.empty()){
        State curr = NODES[pq.top()]; pq.pop();

        if (curr.is_solved()){
            print_path(curr);
            return;
        }

        auto reachable = reachable_positions(curr);

        for (int b = curr.boxes._Find_first(); b < curr.boxes.size(); b = curr.boxes._Find_next(b)){
            for (auto delta : DIR){
                Position from = b - delta;
                Position to = b + delta;

                if (!reachable.test(from)) continue;  // player can not reach position 'from' to push the box
                if (!is_free(to, curr, true)) continue;  // box can not be moved to position 'to'

                State new_state;
                new_state.boxes = curr.boxes;
                new_state.player = b;
                new_state.boxes.reset(b);
                new_state.boxes.set(to);

                if (new_state.is_dead() || visited.count(new_state)) continue;

                new_state.action = {from, delta};
                new_state.compute_heuristic();
                new_state.g_cost = curr.g_cost + 1;
                new_state.idx = NODES.size();
                new_state.parent_idx = curr.idx;

                NODES.push_back(new_state);
                pq.push(new_state.idx);
                visited.insert(new_state);
            }
        }
    }

    cerr << "No solution found.\n";
    exit(1);
}

int main(int argc, char* argv[]){
    /* Set the timer */
    auto start = chrono::high_resolution_clock::now();

    State state = loadState(argv[1]);
    astar_solver(state);

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = end - start;
    ofstream fout("timer.txt", ios::app);
    fout << elapsed.count() << "\n";
    fout.close();

    return 0;
}