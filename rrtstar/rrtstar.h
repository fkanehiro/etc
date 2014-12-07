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

class rrtstar
{
public:
  rrtstar(){
  }

  std::vector<nodePtr>& vertices() { return m_nodes; }
  void initPlan(nodePtr xinit){
    m_nodes.clear();
    m_nodes.push_back(xinit);
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
      m_nodes.push_back(xnew);
      nodePtr xmin = xnearest;
      std::vector<nodePtr> Xnear = near(xnew);
      for (unsigned int i=0; i<Xnear.size(); i++){
	nodePtr xnear = Xnear[i];
	if (obstacleFree(xnear, xnew)){
	  double cdash = xnear->cost() + xnew->cost(xnear);
	  if (cdash < xnew->cost()){
	    xmin = xnear;
	  }
	}
      }
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
      if (x->cost(m_nodes[i]) < 0.2){
	ret.push_back(m_nodes[i]);
      }
    }
    return ret;
  }

  bool obstacleFree(nodePtr from, nodePtr to){
    return true;
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
    return nodePtr(new node((2.0*rand())/RAND_MAX-1.0,
			    (2.0*rand())/RAND_MAX-1.0));
  }

private:
  std::vector<nodePtr> m_nodes;
};
