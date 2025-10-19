#include <iostream>
#define N 5
using namespace std;

// Directions
#define UP 0
#define LEFT 1
#define RIGHT 2
#define DOWN 3

int dx[4] = {-1, 0, 0, 1};
int dy[4] = {0, -1, 1, 0};
int opp[4] = {DOWN, RIGHT, LEFT, UP}; // Opposite direction mapping

int marker[N][N][4]; // path markings

int maze[N][N] = {
    {1, 0, 0, 0, 0},
    {1, 1, 0, 1, 0},
    {0, 1, 1, 1, 0},
    {0, 0, 0, 1, 0},
    {1, 1, 1, 1, 1}
};

// Struct for valid candidates
struct Candidate {
    int dir;
    int nx, ny;
    int mark;
};

int state; // 0 for dead end, 1 for corridor, 2 for junction
Candidate cand[4];  // Global list of candidates (max 4 directions)

// Check valid cell
bool cellisValid(int x, int y) {
    return (x >= 0 && y >= 0 && x < N && y < N);
}

// Check all directions and fill candidates
int direction_check(int x, int y) {
    int nx, ny, n = 0;
    for (int k = 0; k < 4; k++) {
        nx = x + dx[k];
        ny = y + dy[k];
        if (cellisValid(nx, ny) && maze[nx][ny] == 1) {
            cand[n].dir = k;
            cand[n].nx = nx;
            cand[n].ny = ny;
            cand[n].mark = marker[x][y][k];
            n++;
        }
    }
    return n; // number of valid candidates
}

// Choose path based on Tremaux rule
void choose(int &x, int &y, int &came_from) {
    int n = direction_check(x, y);

    if (n == 1) state = 0;       // dead end
    else if (n == 2) state = 1;  // corridor
    else state = 2;              // junction

    cout << "\nAt (" << x << "," << y << ") → State: " 
         << (state == 0 ? "Dead End" : state == 1 ? "Corridor" : "Junction") 
         << endl;

    int unmarked = 0;  //no of candiates that are unmarked for choice making
    for (int i = 0; i < n; i++)
        if (cand[i].mark == 0) unmarked++;

    // --- Choose unmarked path ---
    if (unmarked > 0) {
        for (int i = 0; i < n; i++) {
            // Skip came_from only if valid
            if (cand[i].mark == 0 && (came_from == -1 || cand[i].dir != came_from)) {
                cout << "→ Moving to (" << cand[i].nx << "," << cand[i].ny << ") [Dir " << cand[i].dir << "]\n";
                marker[x][y][cand[i].dir]++;
                marker[cand[i].nx][cand[i].ny][opp[cand[i].dir]]++;
                came_from = opp[cand[i].dir];  //for the next cell
                x = cand[i].nx;
                y = cand[i].ny;
                return;
            }
        }
    } 
    else {
    // All marked (no unmarked): try to backtrack first
    bool moved = false;
    for (int i = 0; i < n; i++) {
        if (came_from != -1 && cand[i].dir == came_from && cand[i].mark < 2) {
            cout << "↩ Backtracking to (" << cand[i].nx << "," << cand[i].ny << ")\n";
            marker[x][y][cand[i].dir]++;
            marker[cand[i].nx][cand[i].ny][opp[cand[i].dir]]++;
            x = cand[i].nx;
            y = cand[i].ny;
            came_from = opp[cand[i].dir];
            moved = true;
            return;
        }
    }

    // Fallback: choose the edge with the smallest mark (<2)
    if (!moved && n > 0) {
        int best = 0;
        for (int i = 1; i < n; i++) {
            if (cand[i].mark < cand[best].mark) best = i;
        }
        cout << "↪ Fallback to (" << cand[best].nx << "," << cand[best].ny 
             << ") [Dir " << cand[best].dir << ", mark " << cand[best].mark << "]\n";
        marker[x][y][cand[best].dir]++;
        marker[cand[best].nx][cand[best].ny][opp[cand[best].dir]]++;
        x = cand[best].nx;
        y = cand[best].ny;
        came_from = opp[cand[best].dir];
        return;
    }
}

}

// Main traversal
int main() {
    int x = 0, y = 0;
    int came_from = -1; // no previous direction initially

    cout << "Starting Tremaux traversal...\n";

    int steps = 0;
    while (!(x == N - 1 && y == N - 1) && steps < 100) { // safety limit to prevent infinite loop
        choose(x, y, came_from);
        steps++;
    }

    if (x == N - 1 && y == N - 1)
        cout << "\n✅ Exit reached at (" << x << "," << y << ")!\n";
        
    else
        cout << "\n❌ Loop limit reached, check marker logic.\n";

    return 0;
}
