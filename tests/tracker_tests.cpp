#include "task3_windmill/tracker.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace task3;
constexpr double pi = 3.14159265358979323846;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
TrackObservation obs(double deg, int index = 0, double quality = 0.8,
                     double distance = 200, double radius = 35)
{ return {index, deg * pi / 180.0, distance, radius, quality}; }
int main()
{
    int passed = 0;
    auto run = [&](const char* name, const std::function<void()>& f)
    {
        f(); ++passed; std::cout << "PASS " << name << '\n';
    };
    try
    {
        run("waiting without observations", [] {
            TargetTracker t; auto r=t.update(true,{});
            check(r.state==TrackingState::WAITING && r.targetId==-1, "waiting"); });
        run("first acquisition", [] {
            TargetTracker t; auto r=t.update(true,{obs(0,4)});
            check(r.targetId==1 && r.selectedIndex==4 && r.hasObservation,"first"); });
        run("do not switch to a brighter second target", [] {
            TargetTracker t; t.update(true,{obs(0)});
            auto r=t.update(true,{obs(72,0,1.0),obs(1,1,0.7)});
            check(r.targetId==1 && r.selectedIndex==1 && r.event=="TRACKED","switch"); });
        run("candidate order can change every frame", [] {
            TargetTracker t; t.update(true,{obs(0,0,0.9),obs(72,1,0.8)});
            for(int i=1;i<=30;++i) {
                const bool reverse=i%2; std::vector<TrackObservation> v;
                if(reverse) v={obs(72+i,0,1),obs(i,1,0.7)};
                else v={obs(i,0,0.7),obs(72+i,1,1)};
                auto r=t.update(true,v);
                check(r.targetId==1 && r.selectedIndex==(reverse?1:0),"order");
            }});
        run("missing selected target does not choose the other one", [] {
            TargetTracker t; t.update(true,{obs(0)});
            auto r=t.update(true,{obs(72)});
            check(r.state==TrackingState::LOST && r.targetId==1 && !r.hasObservation &&
                  r.selectedIndex==-1 && r.locked,"missing"); });
        run("recover before timeout keeps ID", [] {
            TargetTracker t; t.update(true,{obs(0)}); t.update(true,{obs(1)});
            for(int i=0;i<7;++i) t.update(true,{});
            auto r=t.update(true,{obs(9,3),obs(81,4)});
            check(r.targetId==1 && r.event=="REACQUIRED" && r.lostFrames==0,"recover"); });
        run("eighth consecutive failure releases, next frame creates new ID", [] {
            TargetTracker t; t.update(true,{obs(0)});
            for(int i=1;i<=8;++i) {
                auto r=t.update(true,{obs(72)});
                check(r.targetId==1 && !r.hasObservation && r.lostFrames==i,"hold count");
                check(r.locked==(i<8),"release boundary");
                if(i==8) check(r.event=="RELEASED","release event");
            }
            auto r=t.update(true,{obs(72)});
            check(r.targetId==2 && r.event=="ACQUIRED","new id"); });
        run("missing R forbids association and stale output", [] {
            TargetTracker t; t.update(true,{obs(0)});
            auto r=t.update(false,{obs(1)});
            check(r.reason=="R_LOST" && !r.hasObservation && r.selectedIndex==-1,"R lost");
            r=t.update(true,{obs(2)});
            check(r.targetId==1 && r.event=="REACQUIRED","R recovery"); });
        run("ambiguous associations are lost not arbitrary", [] {
            TargetTracker t; t.update(true,{obs(0)});
            auto r=t.update(true,{obs(-1,0),obs(1,1)});
            check(r.reason=="AMBIGUOUS_MATCH" && !r.hasObservation,"ambiguous"); });
        run("wrap around plus and minus 180 degrees", [] {
            TargetTracker t; t.update(true,{obs(179)});
            auto r=t.update(true,{obs(-179)});
            check(r.targetId==1 && r.hasObservation,"wrap");
            check(std::abs(wrapToPi(-358*pi/180)-2*pi/180)<1e-10,"wrap formula"); });
        run("large radius change rejected", [] {
            TargetTracker t; t.update(true,{obs(0)});
            auto r=t.update(true,{obs(1,0,0.8,200,70)});
            check(r.state==TrackingState::LOST,"radius"); });
        run("large orbit distance change rejected", [] {
            TargetTracker t; t.update(true,{obs(0)});
            auto r=t.update(true,{obs(1,0,0.8,350,35)});
            check(r.state==TrackingState::LOST,"distance"); });
        run("nonfinite observation ignored", [] {
            TargetTracker t; auto o=obs(0); o.angleRad=std::numeric_limits<double>::quiet_NaN();
            check(!t.update(true,{o}).hasObservation,"NaN"); });
        run("first choice independent of order when quality ties", [] {
            TargetTracker a,b;
            auto x=a.update(true,{obs(40,0),obs(-40,1)});
            auto y=b.update(true,{obs(-40,0),obs(40,1)});
            check(x.selectedIndex==1 && y.selectedIndex==0,"tie"); });
        run("relative coordinates cancel common image translation", [] {
            auto relative=[](double tx,double ty,double rx,double ry) {
                double dx=tx-rx,dy=ty-ry;
                return TrackObservation{0,std::atan2(-dy,dx),std::hypot(dx,dy),35,0.8};
            };
            TargetTracker t; t.update(true,{relative(700,450,500,400)});
            auto r=t.update(true,{relative(800,480,600,430)});
            check(r.targetId==1 && r.hasObservation && r.angleErrorRad<1e-6,"translation"); });
        run("invalid parameters rejected", [] {
            TrackerParameters p; p.maxLostFrames=0; bool rejected=false;
            try { TargetTracker t(p); } catch(const std::invalid_argument&) { rejected=true; }
            check(rejected,"parameter validation"); });
        std::cout << "ALL " << passed << " TRACKER TESTS PASSED\n";
        return 0;
    }
    catch(const std::exception& e)
    { std::cerr << "FAIL after " << passed << " tests: " << e.what() << '\n'; return 1; }
}
