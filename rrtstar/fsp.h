#include <vector>
#include <cstdio>
#include <limits>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

void rotate2d(double i_x, double i_y, double th, double& o_x, double& o_y)
{
    double costh = cos(th);
    double sinth = sin(th);
    o_x =  costh*i_x - sinth*i_y;
    o_y =  sinth*i_x + costh*i_y;
}

void limitth(double &th)
{
  while (th >  M_PI) th -= 2*M_PI;
  while (th < -M_PI) th += 2*M_PI;
}

class node;

typedef boost::shared_ptr<node> nodePtr;

class node
{
public:
  node(double x, double y, double th) : 
    m_x(x), 
    m_y(y), 
    m_th(th),
    m_cost(0){
    limitth(m_th);
  }
  double x() { return m_x; }
  double y() { return m_y; }
  double th() { return m_th; }
  double cost(){ return m_cost; }
  double cost(nodePtr n){
#if 0
    double dx = m_x - n->x();
    double dy = m_y - n->y();
    double dth = m_th - n->th();
    limitth(dth);
    dth *= 0.1; // weight for rotation
    return sqrt(dx*dx + dy*dy + dth*dth);
#else
    double abs_dx = m_x - n->x();
    double abs_dy = m_y - n->y();
    double rel_dx, rel_dy;
    rotate2d(abs_dx, abs_dy, -n->th(), rel_dx, rel_dy);
    double max_dx = 0.3, max_dy = 0.1, max_dth = 1.0;
    double dth = m_th - n->th();
    limitth(dth);
    double stepx = fabs(rel_dx/max_dx);
    double stepy = fabs(rel_dy/max_dy);
    double stepth = fabs(dth/max_dth);
    return ceil(std::max(stepx, std::max(stepy, stepth)));
#endif
  }
  nodePtr parent() { return m_parent; }
  void setParent(nodePtr n) { 
    m_parent = n; 
    m_cost = m_parent->cost() + cost(n);
  }
  void step(nodePtr n, double& stepx, double& stepy, double& stepth){
    double abs_dx = m_x - n->x();
    double abs_dy = m_y - n->y();
    rotate2d(abs_dx, abs_dy, -n->th(), stepx, stepy);
    stepth = m_th - n->th();
    limitth(stepth);
  } 
private:
  double m_x, m_y, m_th;
  double m_cost;
  nodePtr m_parent;
};

class region
{
public:
  region() :
    m_cx(0),
    m_cy(0),
    m_cth(0),
    m_wx(0),
    m_wy(0),
    m_wth(0){
  }

  region(double cx, double cy, double cth,
	 double wx, double wy, double wth) :
    m_cx(cx),
    m_cy(cy),
    m_cth(cth),
    m_wx(wx),
    m_wy(wy),
    m_wth(wth){
  }

  double cx() { return m_cx; }
  double cy() { return m_cy; }
  double cth() { return m_cth; }
  double wx() { return m_wx; }
  double wy() { return m_wy; }
  double wth() { return m_wth; }

  bool contain(nodePtr n){
    double x = n->x();
    double y = n->y();
    double th = n->th();
    return (x >= m_cx - m_wx/2 && x <= m_cx + m_wx/2
	    && y >= m_cy - m_wy/2 && y <= m_cy + m_wy/2
	    && th >= m_cth - m_wth/2 && th <= m_cth + m_wth/2);
  }
private:
  double m_cx, m_cy, m_cth, m_wx, m_wy, m_wth;
};

class rrtstar
{
public:
  rrtstar() :
    m_searchRegion(0,0,0,2,2,2*M_PI),
    m_gamma(0.2){
  }

  void setGoalRegion(const region& reg) { m_goalRegion = reg; }
  void setSearchRegion(const region & reg) { m_searchRegion = reg; }
  void setGamma(double g) { m_gamma = g; } 

  std::vector<nodePtr>& vertices() { return m_nodes; }
  std::vector<nodePtr>& verticesInGoalRegion() { 
    return m_nodesInGoalRegion;
  }

  void addVertex(nodePtr x){
    m_nodes.push_back(x);
    //std::cout << "new node:" << x->x() << "," << x->y() << "," << x->th() << std::endl;
    if (m_goalRegion.contain(x)){
      m_nodesInGoalRegion.push_back(x);
    }
  }
  void initPlan(nodePtr xinit){
    m_nodes.clear();
    m_nodesInGoalRegion.clear();
    addVertex(xinit);
  }
  void plan(nodePtr xinit, unsigned int niter){
    initPlan(xinit);
    for (int i=0; i<niter; i++){
      oneStep();
    }
  }

  void oneStep(){
    nodePtr xrand = sample();
    extend(xrand);
  }

  void addEdge(nodePtr from, nodePtr to){
    to->setParent(from);
  }
  void extend(nodePtr x){
    nodePtr xnearest  = nearest(x);
    nodePtr xnew = steer(xnearest, x);
    if (obstacleFree(xnearest, xnew)){
      nodePtr xmin = xnearest;
      double cmin = xnearest->cost() + xnew->cost(xnearest);
      std::vector<nodePtr> Xnear = near(xnew);
      for (unsigned int i=0; i<Xnear.size(); i++){
	nodePtr xnear = Xnear[i];
	if (obstacleFree(xnear, xnew)){
	  double c = xnear->cost() + xnew->cost(xnear);
	  if (c < cmin){
	    xmin = xnear;
	    cmin = c;
	  }
	}
      }
      addVertex(xnew);
      addEdge(xmin, xnew);
      for (unsigned int i=0; i<Xnear.size(); i++){
	nodePtr xnear = Xnear[i];
	if (xnear == xmin) continue;
	if (obstacleFree(xnew, xnear) && xnear->cost() > xnew->cost() + xnear->cost(xnew)){
	  nodePtr xparent = xnear->parent();
	  xnear->setParent(xnew);
	}
      }
    }
  }

  std::vector<nodePtr> near(nodePtr x){
    // todo : use kdtree
    std::vector<nodePtr> ret;
    for (unsigned int i=0; i<m_nodes.size(); i++){
      if (x->cost(m_nodes[i]) < m_gamma){
	ret.push_back(m_nodes[i]);
      }
    }
    return ret;
  }

  bool obstacleFree(nodePtr from, nodePtr to){
    if ((from->x() > -0.25 && from->x() < 0.25
	 && from->y() > -0.25 && from->y() < 0.25)
	|| (to->x() > -0.25 && to->x() < 0.25
	    && to->y() > -0.25 && to->y() < 0.25)){
      return false;
    }else{
      return true;
    }
  }

  nodePtr steer(nodePtr from, nodePtr to){
    double abs_dx = to->x() - from->x();
    double abs_dy = to->y() - from->y();
    double dth = to->th() - from->th();
    limitth(dth);
    double rel_dx, rel_dy;
    rotate2d(abs_dx, abs_dy, -from->th(), rel_dx, rel_dy);
    double ratio_dx=1.0, ratio_dy=1.0;
    double max_dx = 0.3, max_dy = 0.1, max_dth = 1.0;
    if (rel_dx >  max_dx) ratio_dx =  max_dx/rel_dx;
    if (rel_dx < -max_dx) ratio_dy = -max_dx/rel_dx;
    if (rel_dy >  max_dy) ratio_dy =  max_dy/rel_dy;
    if (rel_dy < -max_dy) ratio_dy = -max_dy/rel_dy;
    if (dth >  max_dth) dth =  max_dth;
    if (dth < -max_dth) dth = -max_dth;
    double ratio = std::min(ratio_dx, ratio_dy);
    rel_dx *= ratio; 
    rel_dy *= ratio;
    rotate2d(rel_dx, rel_dy, from->th(), abs_dx, abs_dy);
    double th = from->th() + dth;
    limitth(th);
    return nodePtr(new node(from->x() + abs_dx,
			    from->y() + abs_dy,
			    th));
  }

  nodePtr nearest(nodePtr x){
    // todo : use kdtree
    if (!m_nodes.size()) return nodePtr();
    nodePtr nmin = m_nodes[0];
    double cmin = x->cost(nmin);
    for (unsigned int i=1; i<m_nodes.size(); i++){
      double c = x->cost(m_nodes[i]);
      if (c < cmin){
	cmin = c;
	nmin = m_nodes[i];
      }
    }
    return nmin;
  }

  nodePtr sample(){
    return nodePtr(new node((m_searchRegion.wx()*rand())/RAND_MAX
			    + m_searchRegion.cx()-m_searchRegion.wx()/2,
			    (m_searchRegion.wy()*rand())/RAND_MAX
			    + m_searchRegion.cy()-m_searchRegion.wy()/2,
			    (m_searchRegion.wth()*rand())/RAND_MAX 
			    + m_searchRegion.cth()-m_searchRegion.wth()/2));
  }

private:
  std::vector<nodePtr> m_nodes, m_nodesInGoalRegion;
  region m_goalRegion, m_searchRegion;
  double m_gamma;
};
