#include <vector>
#include <hrpPlanner/PathPlanner.h>

class problem
{
public:
    problem(unsigned int dim);
    hrp::BodyPtr addRobot(const std::string& i_name, const std::string &i_url,
                          hrp::BodyPtr i_body=hrp::BodyPtr());
    hrp::BodyPtr addObstacle(const std::string& i_name, const std::string &i_url,
                             hrp::BodyPtr i_body=hrp::BodyPtr());
    hrp::BodyPtr addGoal(const std::string& i_name, const std::string &i_url,
                         hrp::BodyPtr i_body=hrp::BodyPtr());
    void initCollisionCheckPairs(bool checkSelfCollision=true);
    void initOLV(int argc, char *argv[]);
    void initPlanner();
    void updateOLV();
    void updateOLV(const std::vector<hrp::Vector3>& i_starts,
                   const std::vector<hrp::Vector3>& i_ends);
    bool checkCollision();
    PathEngine::PathPlanner *planner() { return & m_planner; }
    void dt(double t) { m_dt = t; }
private:
    hrp::BodyPtr addBody(const std::string& i_name, const std::string &i_url,
                         hrp::BodyPtr i_body=hrp::BodyPtr());

    std::vector<std::string> m_urls, m_names;
    hrp::BodyPtr m_robot, m_goal;
    std::vector<hrp::BodyPtr> m_obstacles, m_bodies;
    PathEngine::PathPlanner m_planner;
    OpenHRP::WorldState wstate;
    OpenHRP::OnlineViewer_var olv;
    double m_dt;
};
