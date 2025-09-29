#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <omp.h>
using namespace std;

/* === Position Structure === */
struct Position{
    int r;
    int c;
    bool operator==(const Position& other)const{
        return r == other.r && c == other.c;
    }

    /* Hash */
    struct Hash{
        size_t operator()(const Position& p)const{
            size_t seed = 0;
            boost::hash_combine(seed, p.r);
            boost::hash_combine(seed, p.c);
            return seed;
        }
    };
};

/* === Global Variables === */
vector<string> MAP;
vector<Position> TARGETS;
int ROWS, COLS;
const unordered_map<char, Position> DIR = {
    {'W', {-1, 0}},
    {'A', {0, -1}},
    {'S', {1, 0}},
    {'D', {0, 1}}
};
vector<vector<vector<int>>> DIST;  // precomputed distances for heuristic

/* === State Structure === */
struct State{
    Position player;
    vector<Position> boxes;
    string path;
    State* parent = nullptr;

    bool operator==(const State& other)const{
        return player == other.player && boxes == other.boxes;
    }

    /* For A star */
    int g_cost = 0;  // cost from start to current state
    int h_cost = 0;  // heuristic cost from current state to goal

    bool operator<(const State& other)const{
        return (g_cost + h_cost) > (other.g_cost + other.h_cost);  // min-heap
    }

    void compute_heuristic(){
        h_cost = 0;
        vector<bool> used(TARGETS.size(), false);
        for (const auto& box : boxes){
            int minDist = INT_MAX, min_idx = -1;
            for (int j = 0; j < (int)TARGETS.size(); j++){
                if (used[j]) continue;
                int dist = 0;
                dist = DIST[j][box.r][box.c];
                if (dist < minDist){
                    minDist = dist;
                    min_idx = j;
                }
            }
            if (min_idx != -1) used[min_idx] = true;
            h_cost += minDist;
        }
    }

    /* Normalization */
    void normalize(){
        sort(
            boxes.begin(), 
            boxes.end(), 
            [](const Position& a, const Position& b){
                return (a.r == b.r) ? (a.c < b.c) : (a.r < b.r);
            }
        );
    }

    /* Game mechanics */
    bool is_solved()const{
        for (const auto& target : TARGETS){
            bool boxOnTarget = false;
            for (const auto& box : boxes){
                if (box == target){
                    boxOnTarget = true;
                    break;
                }
            }
            if (!boxOnTarget) return false;
        }
        return true;
    }

    bool isDeadBox(const Position& box)const{
        if (MAP[box.r][box.c] == '.') return false; // on target

        bool wallUp = (MAP[box.r - 1][box.c] == '#');
        bool wallDown = (MAP[box.r + 1][box.c] == '#');
        bool wallLeft = (MAP[box.r][box.c - 1] == '#');
        bool wallRight = (MAP[box.r][box.c + 1] == '#');
        bool wallUpLeft = (MAP[box.r - 1][box.c - 1] == '#');
        bool wallUpRight = (MAP[box.r - 1][box.c + 1] == '#');
        bool wallDownLeft = (MAP[box.r + 1][box.c - 1] == '#');
        bool wallDownRight = (MAP[box.r + 1][box.c + 1] == '#');

        bool boxUp = false, boxDown = false, boxLeft = false, boxRight = false;
        bool boxUpLeft = false, boxUpRight = false, boxDownLeft = false, boxDownRight = false;
        for (const auto& b : boxes){
            if (b.r == box.r - 1 && b.c == box.c) boxUp = true;
            if (b.r == box.r + 1 && b.c == box.c) boxDown = true;
            if (b.r == box.r && b.c == box.c - 1) boxLeft = true;
            if (b.r == box.r && b.c == box.c + 1) boxRight = true;
            if (b.r == box.r - 1 && b.c == box.c - 1) boxUpLeft = true;
            if (b.r == box.r - 1 && b.c == box.c + 1) boxUpRight = true;
            if (b.r == box.r + 1 && b.c == box.c - 1) boxDownLeft = true;
            if (b.r == box.r + 1 && b.c == box.c + 1) boxDownRight = true;
        }

        /* Two boxes along a wall */
        if (boxUp && ((wallLeft && wallUpLeft) || (wallRight && wallUpRight))) return true;
        if (boxDown && ((wallLeft && wallDownLeft) || (wallRight && wallDownRight))) return true;
        if (boxLeft && ((wallUp && wallUpLeft) || (wallDown && wallDownLeft))) return true;
        if (boxRight && ((wallUp && wallUpRight) || (wallDown && wallDownRight))) return true;

        /* Four boxes in a square */
        if (boxUp && boxLeft && boxUpLeft) return true;
        if (boxUp && boxRight && boxUpRight) return true;
        if (boxDown && boxLeft && boxDownLeft) return true;
        if (boxDown && boxRight && boxDownRight) return true;

        /* Three boxes forming a L-shape in a wall corner */
        if (wallUpLeft && boxUp && boxLeft) return true;
        if (wallUpRight && boxUp && boxRight) return true;
        if (wallDownLeft && boxDown && boxLeft) return true;
        if (wallDownRight && boxDown && boxRight) return true;

        return false;
    }

    bool is_dead()const{
        for (const auto& box : boxes)
            if (isDeadBox(box))
                return true;
        return false;
    }

    /* Hash */
    struct Hash{
        size_t operator()(const State& s)const{
            size_t seed = 0;
            boost::hash_combine(seed, s.player.r);
            boost::hash_combine(seed, s.player.c);
            for (const auto& box : s.boxes){
                boost::hash_combine(seed, box.r);
                boost::hash_combine(seed, box.c);
            }
            return seed;
        }
    };
};

/* === Helper Functions === */
bool is_free(const Position& pos, const State& state, bool isBox = false, bool considerBoxes = true){
    if (pos.r < 0 || pos.r >= ROWS || pos.c < 0 || pos.c >= COLS)
        return false;
    if (MAP[pos.r][pos.c] == '#')
        return false;
    if (MAP[pos.r][pos.c] == '@' && isBox)
        return false;
    if (!considerBoxes) return true;
    for (const auto& box : state.boxes)
        if (box == pos)
            return false;
    return true;
}

void detect_simple_deadlocks(){
    vector<Position> targets;

    /* Find all target positions */
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            if (MAP[r][c] == '.')
                targets.push_back({r, c});

    /* All reachable positions for boxes from target */
    unordered_set<Position, Position::Hash> reachable;

    for (const auto& target : targets){
        queue<Position> q;
        unordered_set<Position, Position::Hash> visited;
        q.push(target);
        visited.insert(target);
        while (!q.empty()){
            Position curr = q.front();
            q.pop();
            reachable.insert(curr);

            for (const auto& dir : {'W', 'A', 'S', 'D'}){
                auto delta = DIR.at(dir);
                Position newPos = {curr.r + delta.r, curr.c + delta.c};
                Position playerPos = {curr.r + delta.r * 2, curr.c + delta.c * 2};
                if (visited.find(newPos) != visited.end()) continue;
                if (!is_free(newPos, State(), true, false)) continue;  // box can not be moved to position 'newPos'
                if (!is_free(playerPos, State(), false, false)) continue;  // player can not stand on position 'playerPos'
                visited.insert(newPos);
                q.push(newPos);
            }
        }
    }

    /* Mark all non-reachable positions as fragile tile */
    for (int r = 0; r < ROWS; r++){
        for (int c = 0; c < COLS; c++){
            Position pos = {r, c};
            if (MAP[r][c] == ' ' && reachable.find(pos) == reachable.end()){
                MAP[r][c] = '@';
            }
        }
    }
}

void compute_dist(){
    DIST.resize(TARGETS.size(), vector<vector<int>>(ROWS, vector<int>(COLS, INT_MAX)));
    for (int i = 0; i < (int)TARGETS.size(); i++){
        queue<Position> q;
        auto target = TARGETS[i];
        q.push(target);
        DIST[i][target.r][target.c] = 0;
        while (!q.empty()){
            Position curr = q.front();
            q.pop();
            for (const auto& dir : {'W', 'A', 'S', 'D'}){
                auto delta = DIR.at(dir);
                Position newPos = {curr.r + delta.r, curr.c + delta.c};
                if (!is_free(newPos, State(), true)) continue;
                if (DIST[i][newPos.r][newPos.c] == INT_MAX || DIST[i][newPos.r][newPos.c] > DIST[i][curr.r][curr.c] + 1){
                    DIST[i][newPos.r][newPos.c] = DIST[i][curr.r][curr.c] + 1;
                    q.push(newPos);
                }
            }
        }
    }
    for (int i = 0; i < (int)TARGETS.size(); i++){
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
    if (!fin){
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }
    
    State init;
    string line;
    int r = 0;
    while (getline(fin, line)){
        MAP.push_back(line);
        for (int c = 0; c < (int)line.size(); c++){
            char cell = line[c];
            if (cell == 'O' || cell == 'X' || cell == '.'){
                TARGETS.push_back({r, c});
            }
            if (cell == 'o'){         // player on regular tile
                init.player = {r, c};
                MAP[r][c] = ' ';
            } else if (cell == 'O'){  // player on target tile
                init.player = {r, c};
                MAP[r][c] = '.';
            } else if (cell == '!'){  // player on fragile tile
                init.player = {r, c};
                MAP[r][c] = '@';
            } else if (cell == 'x'){  // box on regular tile
                init.boxes.push_back({r, c});
                MAP[r][c] = ' ';
            } else if (cell == 'X'){  // box on target tile
                init.boxes.push_back({r, c});
                MAP[r][c] = '.';
            }
        }
        r++;
    }
    fin.close();

    ROWS = MAP.size();
    COLS = MAP[0].size();

    detect_simple_deadlocks();
    compute_dist();

    init.normalize();
    return init;
}

struct ReachResult{
    unordered_set<Position, Position::Hash> reachable;
    unordered_map<Position, Position, Position::Hash> parent;
};

ReachResult reachable_positions(const State& state){
    ReachResult res;
    queue<Position> q;
    q.push(state.player);
    res.reachable.insert(state.player);
    res.parent[state.player] = {-1, -1};  // mark the root

    while (!q.empty()){
        auto curr = q.front();
        q.pop();
        for (const auto& dir : {'W', 'A', 'S', 'D'}){
            auto delta = DIR.at(dir);
            Position newPos = {curr.r + delta.r, curr.c + delta.c};
            if (!is_free(newPos, state)) continue;
            if (res.reachable.find(newPos) == res.reachable.end()){
                res.reachable.insert(newPos);
                res.parent[newPos] = curr;
                q.push(newPos);
            }
        }
    }
    return res;
}

string path_to(const Position& target, const ReachResult& res){
    auto reachable = res.reachable;
    auto parent = res.parent;
    if (reachable.find(target) == reachable.end())
        return "";  // target not reachable
    
    /* Reconstruct path */
    string path;
    for (Position pos = target; parent[pos].r != -1; pos = parent[pos]){
        Position par = parent[pos];
        if (pos.r == par.r + 1 && pos.c == par.c) path += 'S';
        else if (pos.r == par.r - 1 && pos.c == par.c) path += 'W';
        else if (pos.r == par.r && pos.c == par.c + 1) path += 'D';
        else if (pos.r == par.r && pos.c == par.c - 1) path += 'A';
    }
    reverse(path.begin(), path.end());
    return path;
}

void print_path(const State& state){
    string full_path;
    const State* curr = &state;
    while (curr != nullptr){
        full_path = curr->path + full_path;
        curr = curr->parent;
    }
    cout << full_path << "\n";
}

/* === Solvers === */
void astar_solver(State& init){
    priority_queue<State> pq;
    unordered_set<State, State::Hash> visited;

    init.compute_heuristic();
    pq.push(init);
    visited.insert(init);

    while (!pq.empty()){
        State curr = pq.top();
        pq.pop();

        if (curr.is_solved()){
            // print_path(curr);
            cout << curr.path << "\n";
            return;
        }

        auto ret = reachable_positions(curr);

        for (auto& box : curr.boxes){
            for (const auto& dir : {'W', 'A', 'S', 'D'}){
                auto delta = DIR.at(dir);
                Position from = {box.r - delta.r, box.c - delta.c};
                Position to = {box.r + delta.r, box.c + delta.c};

                if (ret.reachable.find(from) == ret.reachable.end()) continue;  // player can not push the box
                if (!is_free(to, curr, true)) continue;  // box can not be moved to position 'to'

                State new_state;
                new_state.boxes = curr.boxes;
                new_state.player = box;  // player moves to the box's current position after pushing
                for (auto& b : new_state.boxes){
                    if (b == box){
                        b = to;
                        break;
                    }
                }
                new_state.normalize();
                if (new_state.is_dead() || visited.find(new_state) != visited.end()) continue;

                new_state.parent = &curr;
                new_state.path = curr.path + path_to(from, ret) + dir;
                // new_state.path = path_to(from, ret) + dir;
                new_state.g_cost = curr.g_cost + 1;
                new_state.compute_heuristic();

                pq.push(new_state);
                visited.insert(new_state);
            }
        }
    }

    cerr << "No solution found.\n";
    exit(1);
}

int main(int argc, char* argv[]){
    if (argc != 2){
        cerr << "Usage: " << argv[0] << " <input_file>\n";
        return 1;
    }

    State state = loadState(argv[1]);
    astar_solver(state);
    return 0;
}