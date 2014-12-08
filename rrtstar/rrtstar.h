#include <vector>
#include <cstdio>
#include <limits>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

class node;

typedef boost::shared_ptr<node> nodePtr;

class node
{
public:
  node(double x, double y) : 
    m_x(x), 
    m_y(y), 
    m_cost(0){
  }
  double x() { return m_x; }
  double y() { return m_y; }
  double cost(){ return m_cost; }
  double cost(nodePtr n){
    double dx = m_x - n->x();
    double dy = m_y - n->y();
    return sqrt(dx*dx + dy*dy);
  }
  nodePtr parent() { return m_parent; }
  void setParent(nodePtr n) { 
    m_parent = n; 
    m_cost = m_parent->cost() + cost(n);
  }
private:
  double m_x, m_y;
  double m_cost;
  nodePtr m_parent;
};

class region
{
public:
  region() :
    m_cx(0),
    m_cy(0),
    m_wx(0),
    m_wy(0){
  }

  region(double cx, double cy,
	 double wx, double wy) :
    m_cx(cx),
    m_cy(cy),
    m_wx(wx),
    m_wy(wy){
  }

  double cx() { return m_cx; }
  double cy() { return m_cy; }
  double wx() { return m_wx; }
  double wy() { return m_wy; }

  bool contain(nodePtr n){
    double x = n->x();
    double y = n->y();
    return (x >= m_cx - m_wx/2 && x <= m_cx + m_wx/2
	    && y >= m_cy - m_wy/2 && y <= m_cy + m_wy/2);
  }
private:
  double m_cx, m_cy, m_wx, m_wy;
};

class rrtstar
{
public:
  rrtstar() :
    m_searchRegion(0,0,2,2),
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
    double c = to->cost(from);
    if (c > 0.1){
      return nodePtr(new node(from->x() + (to->x() - from->x())*0.1/c,
			      from->y() + (to->y() - from->y())*0.1/c));
    }else{
      return to;
    }
  }

  nodePtr nearest(nodePtr x){
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
			    + m_searchRegion.cy()-m_searchRegion.wy()/2));
  }

private:
  std::vector<nodePtr> m_nodes, m_nodesInGoalRegion;
  region m_goalRegion, m_searchRegion;
  double m_gamma;
};
