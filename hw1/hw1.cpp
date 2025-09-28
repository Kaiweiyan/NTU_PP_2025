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

/* === State Structure === */
struct State{
    Position player;
    vector<Position> boxes;
    string path;

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
        for (const auto& box : boxes){
            int minDist = INT_MAX;
            for (const auto& target : TARGETS){
                int dist = 0;
                dist += abs(box.r - target.r) * abs(box.r - target.r);
                dist += abs(box.c - target.c) * abs(box.c - target.c);
                minDist = min(minDist, dist);
            }
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

        /* Corner deadlock */
        if ((wallUp && wallLeft) || (wallUp && wallRight) || (wallDown && wallLeft) || (wallDown && wallRight))
            return true;

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

    init.normalize();
    return init;
}

bool is_free(const Position& pos, const State& state, bool isBox = false){
    if (pos.r < 0 || pos.r >= ROWS || pos.c < 0 || pos.c >= COLS)
        return false;
    if (MAP[pos.r][pos.c] == '#')
        return false;
    if (MAP[pos.r][pos.c] == '@' && isBox)
        return false;
    for (const auto& box : state.boxes)
        if (box == pos)
            return false;
    return true;
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

string path_to(const Position& target, const State& state, const ReachResult& res){
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

/* === Solvers === */
void bfs_solver(const State& init){
    queue<State> q;
    unordered_set<State, State::Hash> visited;

    q.push(init);
    visited.insert(init);

    while (!q.empty()){
        State curr = q.front();
        q.pop();

        if (curr.is_solved()){
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

                State new_state = curr;
                for (auto& b : new_state.boxes){
                    if (b == box){
                        b = to;
                        break;
                    }
                }
                new_state.player = box;  // player moves to the box's original position
                new_state.normalize();
                new_state.path = curr.path + path_to(from, curr, ret) + dir;

                if (!new_state.is_dead() && visited.find(new_state) == visited.end()){
                    visited.insert(new_state);
                    q.push(new_state);
                }
            }
        }
    }

    cerr << "No solution found.\n";
    exit(1);
}

void astar_solver(State& init){
    priority_queue<State> pq;
    unordered_set<State, State::Hash> visited;

    init.compute_heuristic();
    pq.push(init);

    while (!pq.empty()){
        State curr = pq.top();
        pq.pop();

        if (curr.is_solved()){
            cout << curr.path << "\n";
            return;
        }

        if (visited.find(curr) != visited.end()) continue;  // already visited
        visited.insert(curr);

        auto ret = reachable_positions(curr);

        for (auto& box : curr.boxes){
            for (const auto& dir : {'W', 'A', 'S', 'D'}){
                auto delta = DIR.at(dir);
                Position from = {box.r - delta.r, box.c - delta.c};
                Position to = {box.r + delta.r, box.c + delta.c};

                if (ret.reachable.find(from) == ret.reachable.end()) continue;  // player can not push the box
                if (!is_free(to, curr, true)) continue;  // box can not be moved to position 'to'

                State new_state = curr;
                for (auto& b : new_state.boxes){
                    if (b == box){
                        b = to;
                        break;
                    }
                }
                new_state.player = box;  // player moves to the box's original position
                new_state.normalize();
                new_state.path = curr.path + path_to(from, curr, ret) + dir;
                new_state.g_cost = curr.g_cost + 1;
                new_state.compute_heuristic();

                if (!new_state.is_dead() && visited.find(new_state) == visited.end()){
                    pq.push(new_state);
                }
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
    // bfs_solver(state);
    astar_solver(state);
    return 0;
}