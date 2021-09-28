# -*- coding: utf-8 -*-
import lidar
import control as UART
from time import time, sleep
import math
#import matplotlib.pyplot as plt
import argparse
from Mission import MissionManager

# args_parser=argparse.ArgumentParser(description="This script needs control.py and self.LiDAR.py.")
# args_parser.add_argument("-x","--x",metavar="mm",action="store",dest="x",required=True,type=float,help="Set x coordinates for destination. (The units are mm. For example, setting -100 means 100mm to the left.) - Required.")
# args_parser.add_argument("-y","--y",metavar="mm",action="store",dest="y",required=True,type=float,help="Set y coordinates for destination. (The units are mm. For example, setting -100 means 100mm backward.) - Required.")
# args_parser.add_argument("-s","--speed",metavar="30~100",action="store",dest="speed",default=100,type=float,help="Set the speed between 30%% to 100%%. (The 100%% speed is about 5km/h.) - Default : 100%%")
# args_parser.add_argument("-mx","--max_detect_dist",metavar="mm",action="store",default=1500,dest="mx",type=float,help="Set the maximum detection distance for obstacles. (The units are mm. For example, set to detect only obstacles within 1000mm on the driving path if you have set 1000.) - Default : 1500mm / 0 is Unlimited.")
# args_parser.add_argument("-mn","--min_detect_dist",metavar="mm",action="store",default=0,dest="mn",type=float,help="Set the minimum detection distance for obstacles. (The units are mm. For example, set to detect only obstacles behind 1000mm on the driving path if you have set 1000.) - Default : 0mm")
# args_parser.add_argument("-rad","--avoidance_radius",metavar="0~89",action="store",default=15,dest="avoidance_radius",type=float,help="Set the avoidance radius when avoidance driving. (The units are degree. Only values from 0 to 89 can be set.) - Default : 20")
# args_parser.add_argument("-b","--allow_backward",action="store_true",dest="allow_backward",help="If set this flag, allow can move backward as necessary when moving straight.")
# args_parser.add_argument("-r","--reverse_turn",action="store_true",dest="reverse_turn",help="If set this flag, allow can move backward as necessary when turning while avoidance driving.")
# parsed_args=args_parser.parse_args()


# X=parsed_args.x
# Y=parsed_args.y
# SPEED=parsed_args.speed*2.55
# MX=parsed_args.mx
# MN=parsed_args.mn
# ALLOW_BACK=parsed_args.allow_backward
# REVERSE_TURN=parsed_args.reverse_turn
# self.AVD_RAD=abs(parsed_args.avoidance_radius)

class Platform:
    def __init__(self):
        self.X = 1000
        self.Y = 1000
        self.SPEED = 50*2.55
        self.MX = 1500          # max_detect_distance
        self.MN = 0
        self.ALLOW_BACK = True  # 추후에 False로 할수도?
        self.REVERSE_TURN = True
        self.AVD_RAD=15

        self.Gx=self.X
        self.Gy=self.Y
        self.tmpX=self.X
        self.tmpY=self.Y
        self.Cangle=0.00001

        if self.AVD_RAD<0:
            self.AVD_RAD=0
        elif self.AVD_RAD>89:
            self.AVD_RAD=89

        self.mission_num = 0

            
        self.W=346 #width of self.CAR (mm)
        self.H=310 #Length of self.CAR (mm)

        # 차량제어 초기화, 라이다 초기화
        self.CAR=UART.Controller()
        self.LiDAR=lidar.LiDAR()

        # 엔코더 관련변수 초기화
        self.CAR.reset_ENC()
        
    def main(self):    
        print("let's start")
        t=time()
        while time()-t<1:
            self.LiDAR.get()

        # self.mission_num = MissionManager.next_mission()     # 현재 미션키 받아오기

        dst,_=self.set_to_shortcut()
        

        i=0
        while i<len(dst):
            min_dst=0
            if i==len(dst)-1:
                min_dst=self.MN
            colls=self.move_to_dst(dst[i]["angle"],dst[i]["dist"],min_dst=min_dst,speed=self.SPEED)

            dst_list=[]
            max_dst=0
            if colls is not True and colls is not None:
                dst=[]
                near_P=0
                x=0
                y=0
                for coll in colls:
                    x+=coll["x"]
                    y+=coll["y"]
                near_P={"x":x/len(colls),"y":y/len(colls)}

                exit_loop=False
                for interval in range(15,89,15):
                    for j in range(1,-2,-2):
                        x=math.tan(math.radians(interval*j))*near_P["y"]
                        y=(1/math.tan(math.radians(interval*j)))*x

                        _,P=self.set_to_shortcut(x,y)
                        result_P=[]

                        is_colls=False
                        for p in P:
                            post_step={"direction":0,"x":0,"y":0}
                            for step in p:
                                is_colls=self.check_collision(step["angle"],step["dist"],max_dist=0,min_dist=0,start_x=post_step["x"],start_y=post_step["y"],current_angle=post_step["direction"])
                                post_step=step
                                if is_colls is not False:
                                    break
                            if is_colls is not False:
                                break
                            else:
                                # _,dst_P=self.set_to_shortcut(self.tmpX,self.tmpY,allow_CCW=False,current_angle=p[1]["direction"],Cx=p[1]["x"],Cy=p[1]["y"])
                                _,dst_P=self.set_to_shortcut(self.tmpX,self.tmpY,allow_CCW=False,current_angle=p[1]["direction"])
                                for tmpP in dst_P:
                                    post_step=p[1]
                                    for step in tmpP:
                                        tmpMN=0
                                        if step["angle"]==0:
                                            tmpMN=self.MN
                                        is_colls=self.check_collision(step["angle"],step["dist"],max_dist=0,min_dist=tmpMN,current_angle=post_step["direction"],start_x=post_step["x"],start_y=post_step["y"])
                                        post_step=step
                                        if is_colls is not False:
                                            break
                                    if is_colls is not False:
                                        break
                                    else:
                                        cur_dist=0
                                        tmp_dst=p+tmpP
                                        tmp_dst[0]["interval"]=interval*j

                                        dst_list.append(tmp_dst)

                dst_list.sort(key=self.sort_key)
                
                for d in dst_list:
                    dst=d
                    if abs(d[0]["interval"])>=self.AVD_RAD:
                        break

                i=-1
                

            i+=1



    def sort_key(self, dstlist):
        dist=0
        for k in dstlist:
            dist+=abs(k["dist"])
        return dist

    # 최단경로 생성 함수
    def set_to_shortcut(self, Cx=0,Cy=0,abs_steer_angle=25,current_angle=0,allow_backward=False,allow_CCW=True):

        Mx=[math.cos(math.radians(-current_angle))*(self.H/math.tan(math.radians(abs(abs_steer_angle))))+math.sin(math.radians(-current_angle))*self.H+Cx, #Positive coord x
            math.cos(math.radians(-current_angle))*(self.H/math.tan(math.radians(-abs(abs_steer_angle))))+math.sin(math.radians(-current_angle))*self.H+Cx] #Negative coord x
        My=[math.cos(math.radians(-current_angle))*self.H+math.sin(math.radians(-current_angle))*(self.H/math.tan(math.radians(abs(abs_steer_angle))))+Cy, #Positive coord y
            math.cos(math.radians(-current_angle))*self.H+math.sin(math.radians(-current_angle))*(self.H/math.tan(math.radians(-abs(abs_steer_angle))))+Cy] #Negative coord y
        P=[]
        recommanded_P=[]


        cor_Cx=math.sin(math.radians(-current_angle))*self.H+Cx
        cor_Cy=math.cos(math.radians(-current_angle))*self.H+Cy


        for i in range(2):
            steer=abs(abs_steer_angle)
            r=(self.H/math.tan(math.radians(abs(abs_steer_angle))))

            if i>=1:
                r=(self.H/math.tan(math.radians(-abs(abs_steer_angle))))
                steer=-abs(abs_steer_angle)


            C=math.sqrt((self.Gx-Mx[i])**2+(self.Gy-My[i])**2)

            if abs(C)>=abs(r):
                G_theta=math.asin(r/C)
                D=C*math.cos(G_theta)

                alpha=-(Mx[i]-self.Gx)/(My[i]-self.Gy)
                beta=(self.Gx**2-Mx[i]**2+self.Gy**2-My[i]**2+r**2-D**2)/(-2*(My[i]-self.Gy))-self.Gy

                pos_X=(-2*alpha*beta+2*self.Gx+math.sqrt((2*alpha*beta-2*self.Gx)**2-4*(beta**2-D**2+self.Gx**2)*(alpha**2+1)))/(2*alpha**2+2)
                pos_Y=(2*pos_X*(Mx[i]-self.Gx)+self.Gx**2-Mx[i]**2+self.Gy**2-My[i]**2+r**2-D**2)/(-2*(My[i]-self.Gy))

                M=math.sqrt((cor_Cx-pos_X)**2+(cor_Cy-pos_Y)**2)
                L=2*math.asin(M/(2*r))*r

                isForward=True
                if Mx[i]-cor_Cx!=0:
                    isForward = (pos_Y>=(My[i]-cor_Cy)/(Mx[i]-cor_Cx)*(pos_X-Mx[i])+My[i])
                else:
                    isForward = (pos_X>=My[i])

                if current_angle<=270 and current_angle>90:
                    isForward = not isForward

                if not isForward:
                    L*=-1

                directed_D=D
                L_sign=1
                if L!=0:
                    L_sign=L/abs(L)
                P_theta=(2*math.asin(M/(2*r)*L_sign)+math.radians(current_angle))%(math.pi*2)

                Gcos=(self.Gy-pos_Y)/math.sqrt((self.Gx-pos_X)**2+(self.Gy-pos_Y)**2)
                Gsin=(self.Gx-pos_X)/math.sqrt((self.Gx-pos_X)**2+(self.Gy-pos_Y)**2)

                if not (round(math.cos(P_theta),5) == round(Gcos,5) and round(math.sin(P_theta),5) == round(Gsin,5)):
                    if directed_D>0:
                        directed_D*=-1
                else:
                    if directed_D<0:
                        directed_D*=-1

                directed_D-=self.H

                if allow_backward and directed_D<0:
                    if allow_CCW and L<0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])
                    elif L>=0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])
                elif directed_D>=0:
                    if allow_CCW and L<0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])
                    elif L>=0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":pos_X,"y":pos_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])

                neg_X=(-2*alpha*beta+2*self.Gx-math.sqrt((2*alpha*beta-2*self.Gx)**2-4*(beta**2-D**2+self.Gx**2)*(alpha**2+1)))/(2*alpha**2+2) 
                neg_Y=(2*neg_X*(Mx[i]-self.Gx)+self.Gx**2-Mx[i]**2+self.Gy**2-My[i]**2+r**2-D**2)/(-2*(My[i]-self.Gy)) 

                M=math.sqrt((cor_Cx-neg_X)**2+(cor_Cy-neg_Y)**2)
                L=2*math.asin(M/(2*r))*r

                isForward=True
                if Mx[i]-cor_Cx!=0:
                    isForward = (neg_Y>=(My[i]-cor_Cy)/(Mx[i]-cor_Cx)*(neg_X-Mx[i])+My[i])
                else:
                    isForward = (neg_X>=My[i])

                if current_angle<=270 and current_angle>90:
                    isForward = not isForward

                if not isForward:
                    L*=-1

                directed_D=D
                L_sign=1
                if L!=0:
                    L_sign=L/abs(L)

                P_theta=(2*math.asin(M/(2*r)*L_sign)+math.radians(current_angle))%(math.pi*2)

                Gcos=(self.Gy-neg_Y)/math.sqrt((self.Gx-neg_X)**2+(self.Gy-neg_Y)**2)
                Gsin=(self.Gx-neg_X)/math.sqrt((self.Gx-neg_X)**2+(self.Gy-neg_Y)**2)

                if not (round(math.cos(P_theta),5) == round(Gcos,5) and round(math.sin(P_theta),5) == round(Gsin,5)):
                    if directed_D>0:
                        directed_D*=-1
                else:
                    if directed_D<0:
                        directed_D*=-1

                directed_D-=self.H

                if allow_backward and directed_D<0:
                    if allow_CCW and L<0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])
                    elif L>=0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])
                elif directed_D>=0:
                    if allow_CCW and L<0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])
                    elif L>=0:
                        if len(recommanded_P)==0 or abs(recommanded_P[0]["dist"])+abs(recommanded_P[1]["dist"]) > abs(L)+abs(directed_D):
                            recommanded_P=[{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}]
                        P.append([{"x":neg_X,"y":neg_Y,"direction":math.degrees(P_theta),"angle":steer,"dist":L},{"x":self.Gx,"y":self.Gy,"direction":math.degrees(P_theta),"angle":0,"dist":directed_D}])

        return recommanded_P, P

    def set_to_around(self, Cx=0,Cy=0,current_angle=0,allow_backward=False):
        theta=math.acos((self.Gx-Cx)/math.sqrt((self.Gx-Cx)**2+(self.Gy-Cy)**2))
        
        if self.Gy<Cy :
            theta+=math.pi

        theta+=math.radians(current_angle)

        if math.pi/2!=theta%math.pi:
            dist_M=math.sqrt((self.Gx-Cx)**2+(self.Gy-Cy)**2)/2
            radius=dist_M/math.cos(theta)
            steer_angle=math.degrees(math.atan(self.H/radius))

            if allow_backward:
                travel_distance=(math.pi/2-(theta%math.pi))*radius*2
                tmp_dist=(math.pi/2-theta)*radius*2
                if abs(tmp_dist)<abs(travel_distance):
                    travel_distance=tmp_dist
            else:
                travel_distance=(math.pi/2-theta)*radius*2
            return {"angle":steer_angle, "dist":travel_distance}
        else:
            math.radians(current_angle)
            return {"angle":current_angle, "dist":math.sqrt((self.Gx-Cx)**2+(self.Gy-Cy)**2)}

    def calc_Radius(self, angle):
        rad=math.radians(angle)
        if angle>0:
            min_rad=((self.W/2)-(self.H/math.tan(rad)))
            max_rad=math.sqrt((-(self.W/2)-(self.H/math.tan(rad)))**2+self.H**2)
        elif angle<0:
            min_rad=(-(self.W/2)-(self.H/math.tan(rad)))
            max_rad=math.sqrt(((self.W/2)-(self.H/math.tan(rad)))**2+self.H**2)
        else:
            min_rad=-(self.W/2)
            max_rad=self.W/2

        return abs(min_rad), abs(max_rad)

    def check_collision(self, angle,distance,current_angle=0,current_dist=0,start_x=0,start_y=0,max_dist=0,min_dist=1500):
        radius=0
        dist_vertical=False
        cur_vertical=False
        exist=True
        collision_list=[]

        tmp_cur_angle=current_angle

        tmp_cur_angle%=360
        tmp_rad=math.radians(tmp_cur_angle)
        
        tmp_angle=angle
        limitted_distance=distance

        if max_dist!=0 :
            if abs(limitted_distance)>max_dist:
                if limitted_distance<0:
                    limitted_distance=-max_dist
                else:
                    limitted_distance=max_dist
        else:
            if limitted_distance<0:
                limitted_distance-=min_dist
            else:
                limitted_distance+=min_dist
        
        if abs(limitted_distance)<min_dist:
            if limitted_distance<0:
                limitted_distance=-min_dist
            else:
                limitted_distance=min_dist

        if tmp_angle==0:
            for point in self.LiDAR.get_coords():
                exist=True

                x=(point["x"]-start_x)*math.cos(tmp_rad)-(point["y"]-start_y)*math.sin(tmp_rad)+start_x
                y=(point["y"]-start_y)*math.cos(tmp_rad)+(point["x"]-start_x)*math.sin(tmp_rad)+start_y

                exist &= (x>=-(self.W/2)+start_x)
                exist &= (x<=self.W/2+start_x)
                exist &= (y>=start_y+current_dist)
                exist &= (y<=start_y+limitted_distance)

                if exist:
                    collision_list.append(point)
        else:
            radius=self.H/math.tan(math.radians(tmp_angle))

            variation_angle=math.radians((current_dist/radius+tmp_cur_angle)%360)

            if (limitted_distance/radius+variation_angle)%math.pi != math.pi/2:
                dist_alpha=-math.tan(limitted_distance/radius+variation_angle)
            else:
                dist_vertical=True
            
            if (current_dist/radius+variation_angle)%math.pi != math.pi/2:
                cur_alpha=-math.tan(current_dist/radius+variation_angle)
            else:
                cur_vertical=True
            
            min_rad, max_rad = self.calc_Radius(tmp_angle)

            x_axis=(math.cos(-variation_angle)*radius)-(math.sin(-variation_angle)*-self.H)+start_x
            y_axis=(math.sin(-variation_angle)*radius)+(math.cos(-variation_angle)*-self.H)+start_y

            OL_x_axis=(math.cos(-variation_angle)*radius)+start_x
            OL_y_axis=(math.sin(-variation_angle)*radius)+start_y

            isForward = (distance>=0)

            overhalf = (abs(limitted_distance)>=abs(math.pi*radius))

            if dist_vertical==False:
                overline = (start_y>=dist_alpha*(start_x-OL_x_axis)+OL_y_axis)
            else:
                overline = (start_x>=x_axis)

            for point in self.LiDAR.get_coords():
                exist=True
                
                x=point["x"]
                y=point["y"]

                if dist_vertical==False:
                    alpha=dist_alpha
                    if alpha==0:
                        alpha=-1

                    if overline:
                        if overhalf:
                            exist &= (y<=dist_alpha*(x-x_axis)+y_axis)
                        else:
                            exist &= (y>=dist_alpha*(x-x_axis)+y_axis)
                    else:
                        if overhalf:
                            exist &= (y>=dist_alpha*(x-x_axis)+y_axis)
                        else:
                            exist &= (y<=dist_alpha*(x-x_axis)+y_axis)
                else:
                    if overline:
                        if overhalf:
                            exist &= (x<=x_axis)
                        else:
                            exist &= (x>=x_axis)
                    else:
                        if overhalf:
                            exist &= (x>=x_axis)
                        else:
                            exist &= (x<=x_axis)
                
                if cur_vertical==False:
                    alpha=cur_alpha
                    if alpha==0:
                        alpha=-1

                    if tmp_cur_angle<=270 and tmp_cur_angle>=90:
                        if overhalf:
                            if isForward:
                                exist |= (y<=cur_alpha*(x-x_axis)+y_axis)
                            else:
                                exist |= (y>=cur_alpha*(x-x_axis)+y_axis)
                        else:
                            if isForward:
                                exist &= (y<=cur_alpha*(x-x_axis)+y_axis)
                            else:
                                exist &= (y>=cur_alpha*(x-x_axis)+y_axis)
                    else:
                        if overhalf:
                            if isForward:
                                exist |= (y>=cur_alpha*(x-x_axis)+y_axis)
                            else:
                                exist |= (y<=cur_alpha*(x-x_axis)+y_axis)
                        else:
                            if isForward:
                                exist &= (y>=cur_alpha*(x-x_axis)+y_axis)
                            else:
                                exist &= (y<=cur_alpha*(x-x_axis)+y_axis)
                else:
                    if abs(tmp_cur_angle)>=180:
                        if overhalf:
                            if isForward:
                                exist |= (x<=x_axis)
                            else:
                                exist |= (x>=x_axis)
                        else:
                            if isForward:
                                exist &= (x<=x_axis)
                            else:
                                exist &= (x>=x_axis)
                    else:
                        if overhalf:
                            if isForward:
                                exist |= (x>=x_axis)
                            else:
                                exist |= (x<=x_axis)
                        else:
                            if isForward:
                                exist &= (x>=x_axis)
                            else:
                                exist &= (x<=x_axis)
                
                this_rad=math.sqrt((x-x_axis)**2+(y-y_axis)**2)

                exist &= (min_rad<=this_rad)

                exist &= (max_rad>=this_rad)

                if exist:
                    collision_list.append(point)

        if len(collision_list)>0:
            return collision_list
        else:
            return False

    # 목적지까지 이동
    def move_to_dst(self, angle,dist,min_dst, speed):
        # min_dst=self.MN
        if round(abs(dist),0)>0: 
            notice_time=time()
            self.CAR.reset_ENC()
            be_angle=(math.radians(self.Cangle))%(math.pi*2)

            colls=self.check_collision(angle,dist,max_dist=self.MX,min_dist=min_dst)
            restart_delay=1
            rst_time=0

            radius=0
            if angle!=0:
                radius=self.H/math.tan(math.radians(angle))

            while colls!=False:
                if time()-notice_time>1:
                    print("Can't go on calculated path because of collisions.")
                    notice_time=time()
                colls=self.check_collision(angle,dist,max_dist=self.MX,min_dist=min_dst)
                return colls

            if dist>0:
                print("PATHFINDER MOVE MOVE")
                self.CAR.move("GO",speed,angle)
            else:
                self.CAR.move("BACK",speed,angle)

            sleep(0.2)
            std_dist=self.CAR.read("DIST")[0]

            if std_dist==0:
                sleep(0.2)
                std_dist=self.CAR.read("DIST")[0]

            onstop=False
            restart=False

            while True:
                cur_dist=math.copysign(self.CAR.read("DIST")[0]-std_dist,dist)
                colls=self.check_collision(angle,dist-cur_dist,max_dist=self.MX,min_dist=min_dst)

                if radius !=0:
                    cor_theta=(cur_dist/radius)%(math.pi*2)
                    self.Cangle=math.degrees(cor_theta+be_angle)
                    Xc=radius-math.cos(cor_theta)*radius
                    Yc=math.sin(cor_theta)*radius
                    cor_x=math.sin(be_angle)*Yc+math.cos(be_angle)*Xc
                    cor_y=math.sin(be_angle)*Xc+math.cos(be_angle)*Yc
                    Xt=self.Gx-cor_x
                    Yt=self.Gy-cor_y+self.H
                    thetaT=cor_theta
                    self.tmpX=Xt*math.cos(thetaT)-Yt*math.sin(thetaT)
                    self.tmpY=Xt*math.sin(thetaT)+Yt*math.cos(thetaT)-self.H
                else:
                    self.tmpY=self.Gy-cur_dist

                if self.CAR.read("DIST")[0]-std_dist>=abs(dist) :
                    self.CAR.move("STOP")
                    self.CAR.reset_ENC()
                    self.Gx=self.tmpX
                    self.Gy=self.tmpY
                    print("Arrived.")
                    return True
                elif colls!=False:
                    if time()-notice_time>1:
                        print("Can't go on calculated path because of collisions.")
                        notice_time=time()
                    self.CAR.move("STOP")
                    onstop=True
                    return colls
                elif colls==False:
                    if onstop:
                        onstop=False
                        restart=True
                        rst_time=time()
                        print("Waiting for restart.")
                    else:
                        if restart:
                            if time()-rst_time>=restart_delay:
                                if dist>0:
                                    self.CAR.move("GO",speed,angle)
                                else:
                                    self.CAR.move("BACK",speed,angle)
                                print("restart.")
                                restart=False
    '''
    while True:
        print(check_collision(0,1000,max_dist=0,min_dist=0, start_x=0, start_y=0, current_angle=90))
    '''

if __name__ == '__main__':
    print("start")
    P=Platform()
    P.main() 
