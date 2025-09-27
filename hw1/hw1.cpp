#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <boost/functional/hash.hpp>
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

        if ((wallUp && wallLeft) || (wallUp && wallRight) || (wallDown && wallLeft) || (wallDown && wallRight))
            return true;

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

unordered_set<Position, Position::Hash> reachable_positions(const State& state){
    unordered_set<Position, Position::Hash> reachable;
    queue<Position> q;
    q.push(state.player);
    reachable.insert(state.player);

    while (!q.empty()){
        auto curr = q.front();
        q.pop();
        for (const auto& dir : {'W', 'A', 'S', 'D'}){
            auto delta = DIR.at(dir);
            Position newPos = {curr.r + delta.r, curr.c + delta.c};
            if (!is_free(newPos, state)) continue;
            if (reachable.find(newPos) == reachable.end()){
                reachable.insert(newPos);
                q.push(newPos);
            }
        }
    }
    return reachable;
}

string path_to(const State& state, const Position& target){
    unordered_map<Position, Position, Position::Hash> parent;
    queue<Position> q;
    q.push(state.player);
    parent[state.player] = {-1, -1};  // mark the root

    while (!q.empty()){
        auto curr = q.front();
        q.pop();
        if (curr == target) break;

        for (const auto& dir : {'W', 'A', 'S', 'D'}){
            auto delta = DIR.at(dir);
            Position newPos = {curr.r + delta.r, curr.c + delta.c};
            if (!is_free(newPos, state)) continue;
            if (parent.find(newPos) == parent.end()){
                parent[newPos] = curr;
                q.push(newPos);
            }
        }
    }

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

        auto reachable = reachable_positions(curr);

        for (auto& box : curr.boxes){
            for (const auto& dir : {'W', 'A', 'S', 'D'}){
                auto delta = DIR.at(dir);
                Position from = {box.r - delta.r, box.c - delta.c};
                Position to = {box.r + delta.r, box.c + delta.c};

                if (reachable.find(from) == reachable.end()) continue;  // player can not push the box
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
                new_state.path = curr.path + path_to(curr, from) + dir;

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

int main(int argc, char* argv[]){
    if (argc != 2){
        cerr << "Usage: " << argv[0] << " <input_file>\n";
        return 1;
    }

    State state = loadState(argv[1]);
    bfs_solver(state);
    return 0;
}