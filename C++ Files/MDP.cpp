#include "MDP.h"
/**
 * @brief constructor
 * @param alpha transition probability parameter
 * @param dim dimension of grid
 * @param sim simulator pointer
 */
MDP::MDP(double a, int dim, Simulator &sim) {
    this->alpha = a;
    this->N = dim*dim;
    this->simulator = sim;
    this->gridDimension = dim;

    // Arbitararily filling V0 with 0
    std::vector<double> temp;
    values.push_back(temp);
    for(int i =0; i<N; i++) {
        values[0].push_back(1);
    }

    std::vector<Point2D> locs = simulator.getObstacleLocations();
    actions.push_back(MOVE_UP);
    actions.push_back(MOVE_DOWN);
    actions.push_back(MOVE_LEFT);
    actions.push_back(MOVE_RIGHT);
    //Initialize Q-values
    for (int i=0; i<gridDimension; i++) {
        for (int j=0; j<gridDimension; j++) {
            for (auto a: actions) {
                if (std::find(locs.begin(), locs.end(), Point2D(i, j)) != locs.end()) {
                    Q[Point2D(i, j)][a] = -10.0;
                } else if (Point2D(i, j) == simulator.getTarget()) {
                    Q[Point2D(i, j)][a] = 100.0;
                } else {
                    Q[Point2D(i, j)][a] = 0.0;
                }
                
            }
        }
    }
}
/**
 * @brief get reward for transition
 * @param a action
 * @param current currrent state
 * @param next next state
 */
double MDP::getRewardForTransition(RobotAction a, Point2D current, Point2D next) {
    if (next == simulator.getTarget()) return 100;
    else if (next == current) return -10;
    else return -0.04;
    
}
/**
 * @brief get probability of transition
 * @param a robot action
 * @param current current state
 * @param next next state
 * @return probability of transitioning from current astate to next state under action a
 */
double MDP::getProbabilityOfTransition(RobotAction a, Point2D current, Point2D next) {
    if (fabs(current.getX() - next.getX()) > 1 || fabs(current.getY() - next.getY()) > 1 ||
        (fabs(current.getX() - next.getX()) == 1 && fabs(current.getY() - next.getY()) == 1)) {
        return 0.0;
    } else {
        std::vector<Point2D> walls = simulator.getObstacleLocations(current);
        //Order: up, down, left, right
        std::vector<bool> blockedDirections(4, false);
        for (auto w: walls) {
            if (w.getX() == current.getX() - 1 && w.getY() == current.getY()) {
                blockedDirections[0] = true;
            } else if (w.getX() == current.getX() + 1 && w.getY() == current.getY()) {
                blockedDirections[1] = true;
            } else if (w.getX() == current.getX() && w.getY() == current.getY() - 1) {
                blockedDirections[2] = true;
            } else if (w.getX() == current.getX() && w.getY() == current.getY() + 1) {
                blockedDirections[3] = true;
            }
        }
        if (current.getX() == 0) blockedDirections[0] = true;
        if (current.getX() == simulator.getHeight() - 1) blockedDirections[1] = true;
        if (current.getY() == 0) blockedDirections[2] = true;
        if (current.getY() == simulator.getWidth() - 1) blockedDirections[3] = true;
        double tOtherProb = (1.0 - alpha)/3.0;
        int numBlocked = 0;
        for (auto b: blockedDirections) {if (b) numBlocked++;}
        if (a == MOVE_UP) {
            //Intended direction is not blocked
            if (!blockedDirections[0]) {
                if (current.getX() - 1 == next.getX()) {
                    return alpha;
                } else if (current == next){
                    return fmax(0.0, (numBlocked)*tOtherProb);
                } else if (current.getX() + 1 == next.getX() && !blockedDirections[1]) {
                    return tOtherProb;
                } else if (current.getY() - 1 == next.getY() && !blockedDirections[2]) {
                    return tOtherProb;
                } else if (current.getY() + 1 ==  next.getY() && !blockedDirections[3]) {
                    return tOtherProb;
                }
                //Intended direction blocked
            } else {
                if (current == next) {
                    return alpha + (numBlocked - 1)*tOtherProb;
                } else if (current.getX() + 1 == next.getX() && !blockedDirections[1]) {
                    return tOtherProb;
                } else if (current.getY() - 1 == next.getY() && !blockedDirections[2]) {
                    return tOtherProb;
                } else if (current.getY() + 1 ==  next.getY() && !blockedDirections[3]) {
                    return tOtherProb;
                }
            }
        } else if (a == MOVE_DOWN) {
            //Intended direction is not blocked
            if (!blockedDirections[1]) {
                if (current.getX() + 1 == next.getX()) {
                    return alpha;
                } else if (current == next){
                    return fmax(0.0, (numBlocked)*tOtherProb);
                } else if (current.getX() - 1 == next.getX() && !blockedDirections[0]) {
                    return tOtherProb;
                } else if (current.getY() - 1 == next.getY() && !blockedDirections[2]) {
                    return tOtherProb;
                } else if (current.getY() + 1 ==  next.getY() && !blockedDirections[3]) {
                    return tOtherProb;
                }
                //Intended direction blocked
            } else {
                if (current == next) {
                    return alpha + (numBlocked - 1)*tOtherProb;
                } else if (current.getX() - 1 == next.getX() && !blockedDirections[0]) {
                    return tOtherProb;
                } else if (current.getY() - 1 == next.getY() && !blockedDirections[2]) {
                    return tOtherProb;
                } else if (current.getY() + 1 ==  next.getY() && !blockedDirections[3]) {
                    return tOtherProb;
                }
            }
        } else if (a == MOVE_LEFT) {
            //Intended direction is not blocked
            if (!blockedDirections[2]) {
                if (current.getY() - 1 == next.getY()) {
                    return alpha;
                } else if (current == next){
                    return fmax(0.0, (numBlocked)*tOtherProb);
                } else if (current.getX() + 1 == next.getX() && !blockedDirections[1]) {
                    return tOtherProb;
                } else if (current.getX() - 1 == next.getX() && !blockedDirections[0]) {
                    return tOtherProb;
                } else if (current.getY() + 1 ==  next.getY() && !blockedDirections[3]) {
                    return tOtherProb;
                }
                //Intended direction blocked
            } else {
                if (current == next) {
                    return alpha + (numBlocked - 1)*tOtherProb;
                } else if (current.getX() + 1 == next.getX() && !blockedDirections[1]) {
                    return tOtherProb;
                } else if (current.getX() - 1 == next.getX() && !blockedDirections[0]) {
                    return tOtherProb;
                } else if (current.getY() + 1 ==  next.getY() && !blockedDirections[3]) {
                    return tOtherProb;
                }
            }
        } else {
            //Intended direction is not blocked
            if (!blockedDirections[3]) {
                if (current.getY() + 1 == next.getY()) {
                    return alpha;
                } else if (current == next) {
                    return fmax(0.0, (numBlocked)*tOtherProb);
                } else if (current.getX() + 1 == next.getX() && !blockedDirections[1]) {
                    return tOtherProb;
                } else if (current.getX() - 1 == next.getX() && !blockedDirections[0]) {
                    return tOtherProb;
                } else if (current.getY() - 1 ==  next.getY() && !blockedDirections[2]) {
                    return tOtherProb;
                }
                //Intended direction blocked
            } else {
                if (current == next) {
                    return alpha + (numBlocked - 1)*tOtherProb;
                } else if (current.getX() + 1 == next.getX() && !blockedDirections[1]) {
                    return tOtherProb;
                } else if (current.getX() - 1 == next.getX() && !blockedDirections[0]) {
                    return tOtherProb;
                } else if (current.getY() - 1 ==  next.getY() && !blockedDirections[2]) {
                    return tOtherProb;
                }
            }
        }
    }
    return 0.0;
}
/**
 * @brief get transition probability distribution
 * @param a action
 * @param curr current state
 * @return distribution (calculuated) over next state P(N| curr, a)
 */
std::vector<double> MDP::getTransitionDistribution(RobotAction a, Point2D curr) {
    std::vector<double> dist(this->N);
    for (int i=0; i<this->N; i++) {
        int r = i/simulator.getWidth();
        int c = i%simulator.getWidth();
        dist[i] = getProbabilityOfTransition(a, curr, Point2D(r, c));
    }
    return dist;
}
/**
 * @brief value iteration
 * @param valItThreshold error threshold
 * @param discountFactor discount factor
 */
void MDP::valueIteration(double valItThreshold, double gamma) {

    int k = 0;
    
    do {

        k++;

        std::vector<double> temp;
        for(int i = 0; i<N; i++) {
            temp.push_back(0);
        }
        values.push_back(temp);

        std::vector<Point2D> obstacles = this->simulator.getObstacleLocations();
        Point2D target = this->simulator.getTarget();
        
        for(int i=0; i<N; i++) {

            int x = i/gridDimension;
            int y = i%gridDimension;

            bool t = false;

            for(int l = 0; l<obstacles.size(); l++) {
                if(obstacles[l].getX() == x && obstacles[l].getY()==y) {
                    t = true;
                    break;
                }
            }

            if(target.getX()==x && target.getY()==y) {
                t = true;
            }

            if(t) {
                continue;
            }

            // curr state
            Point2D state(x, y);   
            // reachable states from curr
            Point2D up(x-1, y); 
            Point2D down(x+1, y);
            Point2D left(x, y-1);
            Point2D right(x, y+1);
            // Possible states that can be reached from S
            std::vector<Point2D> possibilities;
            possibilities.push_back(up);
            possibilities.push_back(down);
            possibilities.push_back(left);
            possibilities.push_back(right);
            // Temporary vector to store values for the sum for each action
            double sum[4];
            for(int j = 0; j<4; j++) {
                sum[j] = 0;
            }

            // for move up
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[0] += getProbabilityOfTransition(MOVE_UP, state, possibilities[j]) * (getRewardForTransition(MOVE_UP, state, possibilities[j]) + gamma*values[k-1][a]);
                }
            }
            // for move down
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[1] += getProbabilityOfTransition(MOVE_DOWN, state, possibilities[j]) * (getRewardForTransition(MOVE_DOWN, state, possibilities[j]) + gamma*values[k-1][a]);
                }
            }
            // for move left
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[2] += getProbabilityOfTransition(MOVE_LEFT, state, possibilities[j]) * (getRewardForTransition(MOVE_LEFT, state, possibilities[j]) + gamma*values[k-1][a]);
                }
            }
            // for move right
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[3] += getProbabilityOfTransition(MOVE_RIGHT, state, possibilities[j]) * (getRewardForTransition(MOVE_RIGHT, state, possibilities[j]) + gamma*values[k-1][a]);
                }
            }

            // Evaluating the max value for each
            values[k][i] = sum[0];
            for(int j=1; j<4; j++) {
                if(sum[j] > values[k][i]) {     //
                    values[k][i] = sum[j];  // V[k]
                }   
            }

        }        

    } while (condition(k, valItThreshold));

    for(int i=0; i<N; i++) {

            int x = i/gridDimension;
            int y = i%gridDimension;
            // curr state
            Point2D state(x, y);   
            // reachable states from curr
            Point2D up(x-1, y); 
            Point2D down(x+1, y);
            Point2D left(x, y-1);
            Point2D right(x, y+1);

            std::vector<Point2D> possibilities;
            possibilities.push_back(up);
            possibilities.push_back(down);
            possibilities.push_back(left);
            possibilities.push_back(right);
            
            double sum[4];
            for(int j =0; j<4; j++) {
                sum[j] = 0;
            }

             // for move up
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[0] += getProbabilityOfTransition(MOVE_UP, state, possibilities[j]) * (getRewardForTransition(MOVE_UP, state, possibilities[j]) + gamma*values[k][a]);
                }
            }
            // for move down
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[1] += getProbabilityOfTransition(MOVE_DOWN, state, possibilities[j]) * (getRewardForTransition(MOVE_DOWN, state, possibilities[j]) + gamma*values[k][a]);
                }
            }

            double maxsum;
            if(sum[1]>sum[0]) {
                policy[state] = MOVE_DOWN;
                maxsum = sum[1];
            } else {
                policy[state] = MOVE_UP;
                maxsum = sum[0];
            }

            // for move left
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[2] += getProbabilityOfTransition(MOVE_LEFT, state, possibilities[j]) * (getRewardForTransition(MOVE_LEFT, state, possibilities[j]) + gamma*values[k][a]);
                }
            }

            if(sum[2]>maxsum) {
                policy[state] = MOVE_LEFT;
                maxsum = sum[2];
            }
            // for move right
            for(int j = 0; j<4; j++) {
                if(possibilities[j].getX()<gridDimension && possibilities[j].getY()<gridDimension && possibilities[j].getX()>=0 && possibilities[j].getY()>=0) {
                    int a = possibilities[j].getX()*gridDimension + possibilities[j].getY();
                    sum[3] += getProbabilityOfTransition(MOVE_RIGHT, state, possibilities[j]) * (getRewardForTransition(MOVE_RIGHT, state, possibilities[j]) + gamma*values[k][a]);
                }
            }

            if(sum[3]>maxsum) {
                policy[state] = MOVE_RIGHT;
            }

    }

}

// Extra function for checking condition for loop running
bool MDP::condition(int k, double thresh) {
    for(int i = 0; i<N; i++) {
        if(values[k][i] - values[k-1][i] >= thresh) {
            //std::cout<<"problem here: "<<values[k][i] << " : " << values[k-1][i] << std::endl;
            return true;
        }
    }
    return false;
}
/**
 * @bvrief perform q learning update
 * @param a action
 * @param current current state
 * @param next next state
 * @param gamma disccount factor
 * @param learningRate learning rate
 */
void MDP::QLearningUpdate(RobotAction a, Point2D current, Point2D next, double gamma, double learningRate) {
    Q[current][a] = Q[current][a]*(1-learningRate) + learningRate*( getRewardForTransition(a, current, next) + gamma*returnMax(next));
    double max = 0; 
    
    if(Q[current][MOVE_UP] > max) {
        max = Q[current][MOVE_UP];
        policy[current] = MOVE_UP;
    }

    if(Q[current][MOVE_RIGHT] > max) {
        max = Q[current][MOVE_RIGHT];
        policy[current] = MOVE_RIGHT;
    }

     if(Q[current][MOVE_DOWN] > max) {
        max = Q[current][MOVE_DOWN];
        policy[current] = MOVE_DOWN;
    }

     if(Q[current][MOVE_LEFT] > max) {
        max = Q[current][MOVE_LEFT];
        policy[current] = MOVE_LEFT;
    }

}

double MDP::returnMax(Point2D next) {
    double max = 0; 
    
    if(Q[next][MOVE_UP] > max) {
        max = Q[next][MOVE_UP];
    }

    if(Q[next][MOVE_RIGHT] > max) {
        max = Q[next][MOVE_RIGHT];
    }

     if(Q[next][MOVE_DOWN] > max) {
        max = Q[next][MOVE_DOWN];
    }

     if(Q[next][MOVE_LEFT] > max) {
        max = Q[next][MOVE_LEFT];
    }

    return max;
}

/**
 * @brief print value iteration policy
 */
void MDP::printVIPolicy() {
    std::vector<Point2D> locs = simulator.getObstacleLocations();
    for (int x=0; x<this->gridDimension; x++) {
        for (int y=0; y<this->gridDimension; y++) {
            if (std::find(locs.begin(), locs.end(), Point2D(x, y)) != locs.end()) {
                printf("# ");
                continue;
            } else if (Point2D(x, y) == simulator.getTarget()) {
                printf("$ ");
                continue;
            }
            if (policy[Point2D(x, y)] == MOVE_UP) {
                printf("↑ ");
            } else if (policy[Point2D(x, y)] == MOVE_DOWN) {
                printf("↓ ");
            } else if (policy[Point2D(x, y)] == MOVE_LEFT) {
                printf("← ");
            } else {
                printf("→ ");
            }
        }
        printf("\n");
    }
}
/**
 * @brief print q learning policy
 */
void MDP::printQPolicy() {
    std::vector<Point2D> locs = simulator.getObstacleLocations();
    for (int x=0; x<this->gridDimension; x++) {
        for (int y=0; y<this->gridDimension; y++) {
            RobotAction maxAction = getOptimalQLearningAction(Point2D(x, y));
            if (std::find(locs.begin(), locs.end(), Point2D(x, y)) != locs.end()) {
                printf("# ");
                continue;
            } else if (Point2D(x, y) == simulator.getTarget()) {
                printf("$ ");
                continue;
            }
            if (maxAction == MOVE_UP) {
                printf("↑ ");
            } else if (maxAction == MOVE_DOWN) {
                printf("↓ ");
            } else if (maxAction == MOVE_LEFT) {
                printf("← ");
            } else {
                printf("→ ");
            }
        }
        printf("\n");
    }
}
/**
 * @brief get optimal value iteration action
 * @param state state
 * @return optimal value iteration action
 */
RobotAction MDP::getOptimalVIAction(Point2D state) {
    ///TODO: Replace with optimal action from value iteration policy
    return policy[state];
}
/**
 * @brief get optimal q learning action
 * @param state state
 * @return optimal q learning action
 */
RobotAction MDP::getOptimalQLearningAction(Point2D state) {
    ///TODO: Replace with optimal action from Q learning policy
    return policy[state];
}
