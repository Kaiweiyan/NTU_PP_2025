#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
using namespace std;

/* === Position Structure === */
struct Position{
    int r;
    int c;
    bool operator==(const Position& other)const{
        return r == other.r && c == other.c;
    }
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

    bool try_move(char dir, State& new_state)const{
        auto delta = DIR.at(dir);
        Position newPos = {player.r + delta.r, player.c + delta.c};

        if (newPos.r < 0 || newPos.r >= ROWS || newPos.c < 0 || newPos.c >= COLS)
            return false;

        char cell = MAP[newPos.r][newPos.c];
        if (cell == '#')  // wall
            return false;

        bool isBox = false;
        for (const auto& box : boxes){
            if (box == newPos){
                isBox = true;
                break;
            }
        }

        /* Move player to a empty space */
        new_state = *this;
        new_state.player = newPos;
        new_state.path += dir;
        if (!isBox) return true;

        /* Move box */
        Position boxNewPos = {newPos.r + delta.r, newPos.c + delta.c};
        if (boxNewPos.r < 0 || boxNewPos.r >= ROWS || boxNewPos.c < 0 || boxNewPos.c >= COLS)
            return false;
        
        char boxCell = MAP[boxNewPos.r][boxNewPos.c];
        if (boxCell == '#' || boxCell == '@')  // wall or fragile tile
            return false;
        for (const auto& box : boxes)  // another box
            if (box == boxNewPos)
                return false;
        
        /* Valid move */
        for (auto& box : new_state.boxes){
            if (box == newPos){
                box = boxNewPos;
                break;
            }
        }
        new_state.normalize();
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
            size_t hash = (s.player.r << 16) ^ s.player.c;
            for (auto& box : s.boxes)
                hash ^= (box.r * 31 + box.c);
            return hash;
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

        for (const auto& dir : {'W', 'A', 'S', 'D'}){
            State new_state;
            if (curr.try_move(dir, new_state) && visited.find(new_state) == visited.end() && !new_state.is_dead()){
                visited.insert(new_state);
                q.push(new_state);
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