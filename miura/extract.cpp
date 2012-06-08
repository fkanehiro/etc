#include <fstream> 
#include <iostream>
#include <iomanip>
#include <vector>
#include <hrpUtil/Tvmet3d.h>
#include <Math/MathFunction.h>
#include <Interpolator/MotionInterpolator.h>
#include <string.h>

using namespace hrp;
using namespace motion_interpolator;

#define HWIN_SIZE 10

void scale_trajectory(std::vector<Vector3>& trj, double xs, double ys,
                      double zs)
{
    for (unsigned int j=0; j<trj.size(); j++){
        trj[j][0] *= xs;
        trj[j][1] *= ys;
        trj[j][2] *= zs;
    }
}

void translate_trajectory(std::vector<Vector3>& trj, const Vector3 &offset)

{
    for (unsigned int j=0; j<trj.size(); j++){
        trj[j] += offset;
    }
}

// weighted moving average
void wma(const std::vector<Vector3> &i_data, std::vector<Vector3> &o_data,
    unsigned int hwin_size=HWIN_SIZE)
{
    o_data.resize(i_data.size());
    int w, wsum;
    Vector3 v;
    for (unsigned int i=0; i<i_data.size(); i++){
        w = hwin_size+1;
        wsum = w;
        v = w*i_data[i];
        for (unsigned int j=1; j<hwin_size; j++){
            w = hwin_size-j;
            if (j <= i){
                v += w*i_data[i-j];
                wsum += w;
            }
            if (i+j < i_data.size()){
                v += w*i_data[i+j];
                wsum += w;
            }
        }
        v /= wsum;
        o_data[i] = v;
    }
}
 
int main(int argc, char *argv[])
{
    // configuration parameters
    std::string fname;
    double xscale = 1.0;
    double zscale = 1.0;
    double thscale = 1.0;
    double rscale = 1.0;
    double fyoffset = 0.0;
    double fzoffset = 0.0;
    double fyscale = 1.0;
    double fzscale = 1.0;
    double vthd = 0.15;
    // robot dependent parameters
    Vector3 heel2ankle;
    Vector3 ball2ankle;
    Vector3 toe2ball;
    Vector3 ankle2zmp;
    Vector3 toe2zmp;
    //
    const char *conffile = "extract.conf";
    if (argc >= 2){
        if (strcmp("-help", argv[1])==0){
            std::cerr << "Usage: " << argv[0] << "[conf file]" << std::endl;
            return 1;
        }else{
            conffile = argv[1];
        }
    }

    // load configuration file
    std::ifstream conf(conffile);
    if (!conf.is_open()){
        std::cerr << "can't find extra.conf" << std::endl;
        return 1;
    }else{
        std::string token;
        conf >> token;
        while (!conf.eof()){
            if (token == "datafile"){
                conf >> fname;
            }else if(token == "xscale"){
                conf >> xscale;
            }else if(token == "zscale"){
                conf >> zscale;
            }else if(token == "thscale"){
                conf >> thscale;
            }else if(token == "rscale"){
                conf >> rscale;
            }else if(token == "fyoffset"){
                conf >> fyoffset;
            }else if(token == "fzoffset"){
                conf >> fzoffset;
            }else if(token == "fyscale"){
                conf >> fyscale;
            }else if(token == "fzscale"){
                conf >> fzscale;
            }else if(token == "vthd"){
                conf >> vthd;
            }else if(token == "heel2ankle"){
                for (int i=0; i<3; i++) conf >> heel2ankle[i];
            }else if(token == "ball2ankle"){
                for (int i=0; i<3; i++) conf >> ball2ankle[i];
            }else if(token == "toe2ball"){
                for (int i=0; i<3; i++) conf >> toe2ball[i];
            }else if(token == "ankle2zmp"){
                for (int i=0; i<3; i++) conf >> ankle2zmp[i];
            }else if(token == "toe2zmp"){
                for (int i=0; i<3; i++) conf >> toe2zmp[i];
            }else{
                std::cerr << "unknown token(" << token << ")" << std::endl;
            }
            conf >> token;
        }
    }

    // load data from .txt file
    std::ifstream ifs(fname.c_str());
    if (!ifs.is_open()){
        std::cerr << "failed to open(" << fname << ")" << std::endl;
        return 2;
    }

    double time, data;
    Vector3 datav, rhip, lhip;
    std::vector<Vector3> anklejoint[2], toejoint[2], heel[2], toe[2];
    std::vector<Vector3> hip[2], shoulder[2], elbow[2];

    ifs >> time;
    while(!ifs.eof()){
        for (int i=0; i<6; i++) ifs >> data;
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        hip[0].push_back(datav);
        for (int i=0; i<3; i++) ifs >> data;

        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        anklejoint[0].push_back(datav);
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        toejoint[0].push_back(datav);
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        heel[0].push_back(datav);
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        toe[0].push_back(datav);

        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        hip[1].push_back(datav);
        for (int i=0; i<3; i++) ifs >> data;

        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        anklejoint[1].push_back(datav);
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        toejoint[1].push_back(datav);
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; datav[0] *= -1;
        heel[1].push_back(datav);
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; // 41-43
        datav[0] *= -1; toe[1].push_back(datav); 

        for (int i=0; i<21; i++) ifs >> data; // 44-64
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; // 65-67
        datav[0] *= -1; shoulder[0].push_back(datav); 
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; // 68-70
        datav[0] *= -1; elbow[0].push_back(datav); 
        for (int i=0; i<12; i++) ifs >> data; // 71-82
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; // 83-85
        datav[0] *= -1; shoulder[1].push_back(datav); 
        ifs >> datav[1]; ifs >> datav[0]; ifs >> datav[2]; // 86-88
        datav[0] *= -1; elbow[1].push_back(datav); 
        for (int i=0; i<12; i++) ifs >> data; // 89-100
        

        ifs >> time;
    }

    // smoothing
    std::vector<Vector3> smoothed;
    for (int i=0; i<2; i++){
        wma(anklejoint[i], smoothed); anklejoint[i] = smoothed;
        wma(toejoint[i],   smoothed); toejoint[i]   = smoothed;
        wma(heel[i],       smoothed); heel[i]       = smoothed;
        wma(toe[i],        smoothed); toe[i]        = smoothed;
        wma(hip[i],        smoothed); hip[i]        = smoothed;
        wma(shoulder[i],   smoothed); shoulder[i]   = smoothed;
        wma(elbow[i],      smoothed); elbow[i]      = smoothed;
    }


#define DT 0.005
#define HEEL 0x1
#define BALL 0x2
#define TOE  0x4
    // velocities of toe joint, heel and toe and contact
    std::vector<Vector3> toejointv[2], heelv[2], toev[2];
    std::vector<int> contact[2];
    int c = TOE | BALL | HEEL;
    Vector3 v;
    for (int i=0; i<2; i++){
        toejointv[i].push_back(Vector3(0,0,0));
        heelv[i].push_back(Vector3(0,0,0));
        toev[i].push_back(Vector3(0,0,0));
        contact[i].push_back(c);
        for (unsigned int j=1; j<toejoint[i].size(); j++){
            c = 0;
            v = (toejoint[i][j-1] - toejoint[i][j])/DT;
            if (norm2(v) < vthd) c |= BALL;
            toejointv[i].push_back(v);
            v = (heel[i][j-1] - heel[i][j])/DT;
            if (norm2(v) < vthd) c |= HEEL;
            heelv[i].push_back(v);
            v = (toe[i][j-1] - toe[i][j])/DT;
            if (norm2(v) < vthd) c |= TOE;
            toev[i].push_back(v);
            contact[i].push_back(c);
        }
    }

    for (int i=0; i<2; i++){
        scale_trajectory(anklejoint[i], xscale, 1.0, zscale);
        scale_trajectory(toejoint[i], xscale, 1.0, zscale);
        scale_trajectory(heel[i], xscale, 1.0, zscale);
        scale_trajectory(toe[i], xscale, 1.0, zscale);
        scale_trajectory(hip[i], xscale, 1.0, zscale);
        scale_trajectory(shoulder[i], xscale, 1.0, zscale);
        scale_trajectory(elbow[i], xscale, 1.0, zscale);
    }    

    // move start position to the origin
    Vector3 offset;
    offset = -(anklejoint[0][0] + anklejoint[1][0])/2;
    offset[2] = 0;
    for (int i=0; i<2; i++){
        translate_trajectory(anklejoint[i], offset);
        translate_trajectory(toejoint[i], offset);
        translate_trajectory(heel[i], offset);
        translate_trajectory(toe[i], offset);
        translate_trajectory(hip[i], offset);
        translate_trajectory(shoulder[i], offset);
        translate_trajectory(elbow[i], offset);
    }

    // add foot y offset
    offset = 0,fyoffset,0;
    translate_trajectory(anklejoint[1], offset);
    translate_trajectory(toejoint[1], offset);
    translate_trajectory(heel[1], offset);
    translate_trajectory(toe[1], offset);
    translate_trajectory(hip[1], offset);
    offset[1] *= -1;
    translate_trajectory(anklejoint[0], offset);
    translate_trajectory(toejoint[0], offset);
    translate_trajectory(heel[0], offset);
    translate_trajectory(toe[0], offset);
    translate_trajectory(hip[0], offset);

    // adjust ankle height
    double ankleheight = heel2ankle[2];
    for (int i=0; i<2; i++){
        for (unsigned int j=0; j<anklejoint[i].size(); j++){
            double z = anklejoint[i][j][2];
            anklejoint[i][j][2] = (z +fzoffset - ankleheight)*fzscale + ankleheight;
        }
    }

    // waistp and waistr 
    std::vector<Vector3> waistp;
    std::vector<double> waistr;
    for (unsigned int i=0; i<hip[0].size(); i++){
        Vector3 hipcenter((hip[0][i]+hip[1][i])/2);
        Vector3 r2lhip(hip[1][i] - hip[0][i]);
        waistp.push_back(hipcenter);
        waistr.push_back(rscale*atan2(r2lhip[2], r2lhip[1]));
    }

    // angles of foot and toe with respect to the ground
    std::vector<double> footangle[2], toeangle[2];
    for (int j=0; j<2; j++){
        for (unsigned int i=0; i<toejoint[j].size(); i++){
            Vector3 heel2toej(toejoint[j][i] - heel[j][i]);
            footangle[j].push_back(-atan2(heel2toej[2], heel2toej[0]));
            Vector3 toej2toe(toe[j][i] - toejoint[j][i]);
            toeangle[j].push_back(-atan2(toej2toe[2], toej2toe[0]));
        }
    }
    // remove offset angle
    for (int j=0; j<2; j++){
        double footoffset=footangle[j][0], toeoffset=toeangle[j][0];
        for (unsigned int i=0; i<footangle[j].size(); i++){
            footangle[j][i] -= footoffset;
            toeangle[j][i] -= toeoffset;
        }
    }
    

#define sqr(x) ((x)*(x))
#define SOLE_CONTACT(x) (((x)&HEEL)&&((x)&BALL))
#define TOE_CONTACT(x) (((x)&TOE)&&((x)&BALL))
#define CONNECT_NFRAME 50

    for (int i=0; i<2; i++){
        unsigned int start, end, index=0;
        while (1){
            while(index < contact[i].size() && contact[i][index]) index++;
            if (index >= contact[i].size()) break;
            start = index;
            while(!contact[i][index]) index++;
            end = index;
            if (end - start < CONNECT_NFRAME*2) {
                std::cout << "filling small gap" << std::endl;
                for (unsigned int j=start; j<end; j++){
                    contact[i][j] = contact[i][end];
                }
            }
        }
        index = 0;
        while (1){
            while(index < contact[i].size() 
                  && TOE_CONTACT(contact[i][index])) index++;
            if (index >= contact[i].size()) break;
            start = index;
            while(!TOE_CONTACT(contact[i][index])) index++;
            end = index;
            if (end - start < CONNECT_NFRAME*2) {
                std::cout << "filling small gap" << std::endl;
                for (unsigned int j=start; j<end; j++){
                    contact[i][j] = contact[i][end];
                }
            }
        }
    }

    // foot angle and toe angle of the robot
    std::vector<double> footangle_mod[2], toeangle_mod[2];
    for (int i=0; i<2; i++){
        unsigned int start, end, index;
        // foot angle
        index = 0;
        while(1){
            // while sole contact, no change
            while(index < footangle[i].size() 
                  && SOLE_CONTACT(contact[i][index])){
                footangle_mod[i].push_back(0);
                index++;
            }
            if (index >= footangle[i].size()) break; 
            start = index;
            while(!SOLE_CONTACT(contact[i][index])) index++;
            end = index;
            if (end - start < 2*CONNECT_NFRAME){
                std::cerr << "rolling around ball phase is too short(" << (end-start) 
                          << "frames)" << std::endl;
            }
            // smooth connection
            for (unsigned int j=start; j<end; j++){
                double k;
                if (j+1-start < CONNECT_NFRAME){
                    k = polynomial6(((double)(j+1-start))/CONNECT_NFRAME);
                }else if (end - j < CONNECT_NFRAME){
                    k = polynomial6(((double)(end - j))/CONNECT_NFRAME);
                }else{
                    k = 1.0;
                }
                footangle_mod[i].push_back(thscale*k*footangle[i][j]);
            }
        }

        // toe angle
        index = 0;
        while(1){
            // while toe contact, no change 
            while(index < toeangle[i].size() 
                  && TOE_CONTACT(contact[i][index])){
                toeangle_mod[i].push_back(0);
                index++;
            }
            if (index >= toeangle[i].size()) break;
            start = index;
            while(!TOE_CONTACT(contact[i][index])) index++;
            end = index;
            if (end - start < 2*CONNECT_NFRAME){
                std::cerr << "rolling around toe phase is too short(" << (end-start) 
                          << "frames)" << std::endl;
            }
            // smooth connection
            for (unsigned int j=start; j<end; j++){
                double k;
                if (j+1-start < CONNECT_NFRAME){
                    k = polynomial6(((double)(j+1-start))/CONNECT_NFRAME);
                }else if (end - j < CONNECT_NFRAME){
                    k = polynomial6(((double)(end - j))/CONNECT_NFRAME);
                }else{
                    k = 1.0;
                }
                toeangle_mod[i].push_back(thscale*k*toeangle[i][j]);
            }
        }
    }

    // foot and toe attitude of the robot
    std::vector<Matrix33> footR[2], toeR[2];
    Vector3 yaxis(0,1,0);
    for (int i=0; i<2; i++){
        for (unsigned int j=0; j<footangle_mod[i].size(); j++){
            footR[i].push_back(rodrigues(yaxis, footangle_mod[i][j]));
            toeR[i].push_back(rodrigues(yaxis, toeangle_mod[i][j]));
        }
    }

    // ankle and toe joints positions of the robot
    std::vector<Vector3> anklejoint_mod[2], toejoint_mod[2];
    Vector3 heel2ball(heel2ankle - ball2ankle);
    for (int i=0; i<2; i++){
        unsigned int index=0, contactindex=0;
        Vector3 heelp, ball, tiptoe, ankle;
        unsigned int start, end;
        double v, a;
        while(1){
            contactindex = index;
            if (contact[i][contactindex] & HEEL){
                // heel contact 
                heelp[0] = heel[i][contactindex][0];
                heelp[1] = heel[i][contactindex][1];
                heelp[2] = 0;
            }else if (contact[i][contactindex] & BALL){
                ball[0] = toejoint[i][contactindex][0];
                ball[1] = toejoint[i][contactindex][1];
                ball[2] = toe2ball[2];
            }
            while (index < contact[i].size() && contact[i][index] & HEEL){
                anklejoint_mod[i].push_back(Vector3(heelp + footR[i][index]*heel2ankle));
                ball = heelp + footR[i][index]*heel2ball;
                toejoint_mod[i].push_back(ball);
                index++;
            }
	    // ball contact
            while (index < contact[i].size() && contact[i][index] & BALL){
                anklejoint_mod[i].push_back(Vector3(ball +  footR[i][index]*ball2ankle));
                toejoint_mod[i].push_back(ball);
                tiptoe = ball - toeR[i][index]*toe2ball;
                index++;
            }
            // toe contact
            while (index < contact[i].size() && contact[i][index] & TOE){
                ball = tiptoe + toeR[i][index]*toe2ball;
                toejoint_mod[i].push_back(ball);
                anklejoint_mod[i].push_back(Vector3(ball
                                                    + footR[i][index]*ball2ankle));
                index++;
            }
            // in the air
            while(index < contact[i].size() && !contact[i][index]){
                ankle = anklejoint[i][index];
                ball = ankle - footR[i][index]*ball2ankle;
                anklejoint_mod[i].push_back(ankle);
                toejoint_mod[i].push_back(ball);
                index++;
            }
            if (index >= contact[i].size()) break;
        }

#if 1
        index = 0;
        while(1){
            while(index < contact[i].size() && contact[i][index]) index++;
            if (index >= contact[i].size()) break;
            start = index-1; // the last frame of the contacting phase
            while (!contact[i][index]) index++;
            end = index; // the first frame of the contacting phase
            if (end - start < 2*CONNECT_NFRAME){
                std::cerr << "swing phase is too short(" << (end-start) 
                          << "frames)" << std::endl;
            }
            // smooth connection
            MotionInterpolator rh[] = {MotionInterpolator(POINT_TO_POINT),
                                       MotionInterpolator(POINT_TO_POINT),
                                       MotionInterpolator(POINT_TO_POINT)};
            MotionInterpolator hr[] = {MotionInterpolator(POINT_TO_POINT),
                                       MotionInterpolator(POINT_TO_POINT),
                                       MotionInterpolator(POINT_TO_POINT)};
            // foot y scale 
            double ystart = anklejoint_mod[i][start-1][1];
            double yend   = anklejoint_mod[i][  end+1][1];
            for (unsigned int j=start-1; j<=end+1; j++){
                double yref = ystart + ((yend - ystart)*(j-(start-1)))/(end+1 - (start-1));
                double y = anklejoint[i][j][1];
                double dy = y - yref;
                anklejoint[i][j][1] = yref + dy*fyscale;
            }
            Vector3 ajstart(anklejoint[i][start]);
            Vector3 ajend(anklejoint[i][end]);
            Vector3 dstart(anklejoint_mod[i][start] - ajstart);
            Vector3 vrobot, vhuman;
            vrobot = (anklejoint_mod[i][start] - anklejoint_mod[i][start-1])/DT;
            vhuman = (anklejoint[i][start] - anklejoint[i][start-1])/DT;
            Vector3 dstartv( vrobot - vhuman );
            Vector3 dend(anklejoint_mod[i][end] - ajend);
            vrobot = (anklejoint_mod[i][end+1] - anklejoint_mod[i][end])/DT;
            vhuman = (anklejoint[i][end+1] - anklejoint[i][end])/DT;
            Vector3 dendv( vrobot - vhuman);
            Vector3 dp;
            for (int j=0; j<3; j++){
                rh[j].setMotionData(start, dstart[j], dstartv[j],0.0,
                                    start+CONNECT_NFRAME,0.0);
                hr[j].setMotionData(end-CONNECT_NFRAME, dend[j], -dendv[j],0.0,
                                    end, 0.0);
            }
#if 1
            for (unsigned int j=start; j<end; j++){
                if (j < start+CONNECT_NFRAME){
                    for (int k=0; k<3; k++)rh[k].getMotionData(j, dp[k], v, a);
                }else if (j < end-CONNECT_NFRAME){
                    dp = 0,0,0;
                }else{
                    for (int k=0; k<3; k++){
                        hr[k].getMotionData(
                            end - (j - (end-CONNECT_NFRAME)), dp[k], v, a);
                    }
                }
                anklejoint_mod[i][j] = Vector3(anklejoint[i][j]+dp);
                ball = anklejoint_mod[i][j] - footR[i][j]*ball2ankle;
                toejoint_mod[i][j] = ball;
            }
#endif
        }
#endif
    }

    // check toe penetration
#if 1
    for (int i=0; i<2; i++){
        Vector3 tiptoe;
        bool adjusting=false;
        for (unsigned int j=0; j<anklejoint_mod[i].size(); j++){
            Vector3 &ankle = anklejoint_mod[i][j];
            Vector3 & ball = toejoint_mod[i][j];
            tiptoe = ball - toeR[i][j]*toe2ball;
            if (tiptoe[2] < -1e-5){
                if (!adjusting) std::cout << j*DT << std::endl;
                adjusting = true;
                ankle[2] -= tiptoe[2];
                ball[2] -= tiptoe[2];
            }else{
                if (adjusting){ // smooth connection
                    adjusting = false;
                    MotionInterpolator mi = MotionInterpolator(POINT_TO_POINT);
                    double zstart, zend, vzstart, vzend;
                    zstart = anklejoint_mod[i][j-1][2];
                    vzstart = (anklejoint_mod[i][j-1][2] - anklejoint_mod[i][j-2][2])/DT;
                    zend = anklejoint_mod[i][j-1+CONNECT_NFRAME][2];
                    vzend = (anklejoint_mod[i][j-1+CONNECT_NFRAME][2] - anklejoint_mod[i][j-2+CONNECT_NFRAME][2])/DT;
                    mi.setMotionData(j-1, zstart, vzstart,0.0,
                                     j-1+CONNECT_NFRAME, zend, vzend);
                    double p,v,a;
                    for (unsigned int k=j-1; k<j-1+CONNECT_NFRAME; k++){
                        mi.getMotionData(k, p, v, a);
                        toejoint_mod[i][k][2] += p - anklejoint_mod[i][k][2];
                        anklejoint_mod[i][k][2] = p;
                    }
                }
            }
        }
    }
#endif

    // zmp
    time = 0;
    std::vector<std::pair<double, Vector3> > zmpkey;
    int postype = 0;
    Vector3 zmp((anklejoint_mod[0][0] + anklejoint_mod[1][0]+2*ankle2zmp)/2);
    zmpkey.push_back(std::make_pair(time, zmp));
    for (unsigned int i=1; i<contact[0].size(); i++){
        if (SOLE_CONTACT(contact[0][i]) && SOLE_CONTACT(contact[1][i])){
            if (postype != 0){
                postype = 0;
                zmp = (anklejoint_mod[0][i] + anklejoint_mod[1][i]+2*ankle2zmp)/2;
                zmpkey.push_back(std::make_pair(time, zmp));
            }
        }else if (SOLE_CONTACT(contact[0][i]) && !SOLE_CONTACT(contact[1][i])){
            if (postype != 1){
                postype = 1;
                zmp = anklejoint_mod[0][i] + ankle2zmp;
                if (zmpkey.size() == 1){
                    zmpkey.push_back(std::make_pair(time-0.1, zmpkey[0].second));
                }
                zmpkey.push_back(std::make_pair(time, zmp));
#if 1
            }else if (i>0 && !contact[1][i-1]){ // touch down 
                zmp = anklejoint_mod[0][i] + ankle2zmp;
                zmpkey.push_back(std::make_pair(time, zmp));
#endif
            }
        }else if (TOE_CONTACT(contact[0][i]) && !SOLE_CONTACT(contact[1][i])){
#if 0
            if (postype != 2){
                postype = 2;
                zmp = toejoint_mod[0][i] + toe2zmp;
                zmpkey.push_back(std::make_pair(time, zmp));
            }
#endif
        }else if (!SOLE_CONTACT(contact[0][i]) && SOLE_CONTACT(contact[1][i])){
            if (postype != 3){
                postype = 3;
                zmp = anklejoint_mod[1][i] + ankle2zmp;
                if (zmpkey.size() == 1){
                    zmpkey.push_back(std::make_pair(time-0.1, zmpkey[0].second));
                }
                zmpkey.push_back(std::make_pair(time, zmp));
#if 1
            }else if (i>0 && !contact[0][i-1]){ // touch down 
                zmp = anklejoint_mod[1][i] + ankle2zmp;
                zmpkey.push_back(std::make_pair(time, zmp));
#endif
            }
        }else if (!SOLE_CONTACT(contact[0][i]) && TOE_CONTACT(contact[1][i])){
#if 0
            if (postype != 4){
                postype = 4;
                zmp = toejoint_mod[1][i] + toe2zmp;
                zmpkey.push_back(std::make_pair(time, zmp));
            }
#endif
        }else if (contact[0][i] == HEEL && contact[1][i] == TOE){
#if 0
            if (postype != 5){
                postype = 5;
                std::cout << i*DT << std::endl;
                zmp = toejoint_mod[1][i] - toeR[1][i]*toe2ball;
                zmpkey.push_back(std::make_pair(time, zmp));
            }
#endif
        }else{
            std::cerr << "unsupported contact combination(t = "
                      << i*DT << ","
                      << contact[0][i] << "," << contact[1][i] << ")" 
                      << std::endl;
        }
        time+=DT;
    }
    zmp = (anklejoint_mod[0][anklejoint_mod[0].size()-1] 
           + anklejoint_mod[1][anklejoint_mod[1].size()-1]
           +2*ankle2zmp)/2;
    zmpkey.push_back(std::make_pair(time-DT, zmp));

    
    const char *fnames[] = {"trj.rfoot", "trj.lfoot"}; 
    for (int k=0; k<2; k++){
        std::ofstream ofs(fnames[k]);
        time = 0;
        for (unsigned int i=0; i<toejoint[k].size(); i++){
            // time
            ofs << time << " "; // 1

            // position
            for (int j=0; j<3; j++) ofs << toejoint[k][i][j] << " "; // 2-4
            for (int j=0; j<3; j++) ofs << heel[k][i][j] << " "; // 5-7
            for (int j=0; j<3; j++) ofs << toe[k][i][j] << " "; // 8-10

            // velocity
            for (int j=0; j<3; j++) ofs << toejointv[k][i][j] << " "; // 11-13
            for (int j=0; j<3; j++) ofs << heelv[k][i][j] << " "; // 14-16
            for (int j=0; j<3; j++) ofs << toev[k][i][j] << " "; // 17-19

            // velocity norm
            ofs << norm2(toejointv[k][i]) << " " << norm2(heelv[k][i]) << " " 
                << norm2(toev[k][i]) << " "; // 20-22

            // angle
            ofs << footangle[k][i] << " " << toeangle[k][i] << " "; // 23, 24

            // modified angle
            ofs << footangle_mod[k][i] << " " << toeangle_mod[k][i] << " "; // 25, 26

            ofs << std::endl;
            time += DT;
        }
    }

    // contact phase information
    std::ofstream phase("trj.phase");
    time = 0;
    for (unsigned int i=0; i<contact[0].size(); i++){
        phase << time  << " ";
        for (int j=0; j<2; j++) phase << contact[j][i] << " ";
        phase << (contact[0][i] && contact[1][i]) << " ";
        phase << std::endl;
        time += DT;
    }

    // ankle positions of human and robot
    const char *anklefname[] = {"trj.rankle", "trj.lankle"};
    for (int i=0; i<2; i++){
        std::ofstream ofs(anklefname[i]);
        time = 0;
        for (unsigned int j=0; j<anklejoint[i].size(); j++){
            ofs << time << " "; // 1

            for (int k=0; k<3; k++) ofs << anklejoint[i][j][k] << " ";//2-4
            for (int k=0; k<3; k++) ofs << anklejoint_mod[i][j][k] << " ";//5-7

            if (j==0){
                ofs << "0 0 0 ";
            }else{
                Vector3 v((anklejoint_mod[i][j] - anklejoint_mod[i][j-1])/DT);
                for (int k=0; k<3; k++) ofs << v[k] << " ";
            }

            ofs << std::endl;
            time += DT;
        }
    }

    // toe positions of human and robot
    const char *toefname[] = {"trj.rtoe", "trj.ltoe"};
    for (int i=0; i<2; i++){
        std::ofstream ofs(toefname[i]);
        Vector3 ballp, toep;
        time = 0;
        for (unsigned int j=0; j<toejoint[i].size(); j++){
            ofs << time << " "; // 1

            for (int k=0; k<3; k++) ofs << toe[i][j][k] << " ";//2-4

            ballp = anklejoint_mod[i][j] - footR[i][j]*ball2ankle;
            toep = ballp - toeR[i][j]*toe2ball; 

            for (int k=0; k<3; k++) ofs << toep[k] << " ";//5-7

            ofs << std::endl;
            time += DT;
        }
    }

    // heel positions of human and robot
    const char *heelfname[] = {"trj.rheel", "trj.lheel"};
    for (int i=0; i<2; i++){
        std::ofstream ofs(heelfname[i]);
        Vector3 heelp;
        time = 0;
        for (unsigned int j=0; j<heel[i].size(); j++){
            ofs << time << " "; // 1

            for (int k=0; k<3; k++) ofs << heel[i][j][k] << " ";//2-4

            heelp = anklejoint_mod[i][j] - footR[i][j]*heel2ankle;

            for (int k=0; k<3; k++) ofs << heelp[k] << " ";//5-7

            ofs << std::endl;
            time += DT;
        }
    }

    // .contact
    std::ofstream contactf("trj.contact");
    contactf << std::setprecision(10);
    contactf << "#Time ncontact linkname relx rely relz relR00 ... relR22 x y z R00 ... R22 linkname relx rely relz ..." << std::endl;
    time = 0;
    for (unsigned int i=0; i<footR[0].size(); i++){
        contactf << time << " 4 "; // 1,2
        contactf << "R_ANKLE_R 0 0 0 1 0 0 0 1 0 0 0 1 "; // 3-15
        for (int j=0; j<3; j++) contactf << anklejoint_mod[0][i][j] << " ";//16-18
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                contactf << footR[0][i](j,k) << " ";//19-27
            }
        }
        contactf << "R_TOE_P 0 0 0 1 0 0 0 1 0 0 0 1 ";//28-40
        for (int j=0; j<3; j++) contactf << toejoint_mod[0][i][j] << " ";//41-43
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                contactf << toeR[0][i](j,k) << " ";
            }
        }
        contactf << "L_ANKLE_R 0 0 0 1 0 0 0 1 0 0 0 1 ";
        for (int j=0; j<3; j++) contactf << anklejoint_mod[1][i][j] << " ";
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                contactf << footR[1][i](j,k) << " ";
            }
        }
        contactf << "L_TOE_P 0 0 0 1 0 0 0 1 0 0 0 1 ";
        for (int j=0; j<3; j++) contactf << toejoint_mod[1][i][j] << " ";
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                contactf << toeR[1][i](j,k) << " ";
            }
        }
        contactf << std::endl;
        time += DT;
    }

    // .waist
    time = 0;
    std::ofstream waistf("trj.waist");
    waistf << std::setprecision(10);
    Vector3 xaxis(1,0,0);
    for (unsigned int i=0; i<waistp.size(); i++){
        waistf << time << " ";
        for (int j=0; j<3; j++) waistf << waistp[i][j] << " ";
        Matrix33 waistR = rodrigues(xaxis, waistr[i]);
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                waistf << waistR(j,k) << " ";
            }
        }
        if (time == 0){
            waistf << "0 0 0 "; 
            waistf << "0 0 0 "; 
        }else{
            Vector3 v((waistp[i] - waistp[i-1])/DT);
            for (int j=0; j<3; j++) waistf << v[j] << " ";
            waistf << (waistr[i] - waistr[i-1])/DT << " 0 0 "; 
        }

        waistf << std::endl;
        time += DT;
    }

    // .pos
    time = 0;
    std::ofstream posf("trj.pos");
    posf << std::setprecision(10);
    for (unsigned int i=0; i<waistr.size(); i++){
        posf << time << " ";
        // legs
        for (int j=0; j<6; j++) posf << "0 ";
        posf << (toeangle_mod[0][i] - footangle_mod[0][i]) << " ";
        for (int j=0; j<6; j++) posf << "0 ";
        posf << (toeangle_mod[1][i] - footangle_mod[1][i]) << " ";
        // waist
        posf << "0 ";
        posf << -waistr[i] << " 0  ";
        // neck + face
        for (int j=0; j<11; j++) posf << "0 ";
        Vector3 se;
        se = elbow[0][i] - shoulder[0][i];
        posf << atan2(-se[0], -se[2]) << " "; // r-shoulder-p
        posf << "-0.2 "; // r-shoulder-r
        for (int j=0; j<6; j++) posf << "0 ";
        se = elbow[1][i] - shoulder[1][i];
        posf << atan2(-se[0], -se[2]) << " ";
        posf << "0.2 "; // l-shoulder-r
        for (int j=0; j<6; j++) posf << "0 ";

        posf << std::endl;
        time += DT; 
    }

    // .zmp
    time = 0;
    std::ofstream zmpf("trj.zmpref");
    zmpf << std::setprecision(10);
    zmpf << "#Time Xzmp_model YZmp_model Zzmp_model Xzmp_cmd YZmp_cmd Zzmp_cmd Xzmp_ideal YZmp_ideal Zzmp_ideal" << std::endl;
    unsigned int segment = 0;
    while (segment < zmpkey.size()-1){
        double dt = zmpkey[segment+1].first - zmpkey[segment].first;
        while (time < zmpkey[segment+1].first){
            zmpf << time << " 0 0 0 0 0 0 ";
            double ratio = polynomial3((time - zmpkey[segment].first)/dt);
            for (int j=0; j<3; j++){
                zmp[j] = ratio*zmpkey[segment+1].second[j] 
                    + (1-ratio)*zmpkey[segment].second[j];
                zmpf << zmp[j] << " ";
            }
            zmpf << std::endl;
            time += DT;
        }
        segment++;
    }
    

    return 0;
}
