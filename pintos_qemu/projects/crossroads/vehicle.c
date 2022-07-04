#include <stdio.h>
#include <stdlib.h>
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

struct lock *step;
struct semaphore *flag;
struct semaphore *threads;
struct semaphore *step_check;
struct semaphore *intersection_whole;
struct semaphore *intersection;
struct semaphore *intersection_out;
int __thread_cnt, current_thread_num, flag_1, flag_crossroad[4], flag_2, out_intersection[5][2];
char flag_crossroad_2[4];
/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][10] = {
   /* from A */ {
      /* to A */
      {{-1,-1},},
      /* to B */
      {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
      /* to C */
      {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
      /* to D */
      {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
   },
   /* from B */ {
      /* to A */
      {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
      /* to B */
      {{-1,-1},},
      /* to C */
      {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
      /* to D */
      {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
   },
   /* from C */ {
      /* to A */
      {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
      /* to B */
      {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
      /* to C */
      {{-1,-1},},
      /* to D */
      {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
   },
   /* from D */ {
      /* to A */
      {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
      /* to B */
      {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
      /* to C */
      {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
      /* to D */
      {{-1,-1},}
   }
};

const int intersections_path[4][4][4]={
   //[i][j][0] : num of value
   //[i][j][1]~ : path that passing intersection of course of i(start)->j(dest)
   /*from A*/{
      /*to A*/
      {0},
      /*to B*/
      {1, 0},
      /*to C*/
      {2, 0,1},
      /*to D*/
      {3, 0,1,2}
   },
   /*from B*/{
      /*to A*/
      {3, 1,2,3},
      /*to B*/
      {0},
      /*to C*/
      {1, 1},
      /*to D*/
      {2, 1,2}
   },
   /*from C*/{
      /*to A*/
      {2, 2,3},
      /*to B*/
      {3, 0,2,3},
      /*to C*/
      {0},
      /*to D*/
      {1, 2}
   },
   /*from D*/{
      /*to A*/
      {1, 3},
      /*to B*/
      {2, 0,3},
      /*to C*/
      {3, 0,1,3},
      /*to D*/
      {0}
   }
};

static int is_position_outside(struct position pos)
{
   return (pos.row == -1 || pos.col == -1);
}

static int is_position_intersection(struct position pos)
{
   return ((pos.row==2 && pos.col>=2 && pos.col<=4)
         || (pos.row==3 && (pos.col==2 || pos.col==4))
         || (pos.row==4 && pos.col>=2 && pos.col<=4));
}

void position_initialize(int start, int dest){
   int num=intersections_path[start][dest][0];
   int tmp=0;
   for(int i=1; i<num+1; i++){ //release intersection that used
      tmp=intersections_path[start][dest][i];
      sema_down(&intersection[tmp]);
      flag_crossroad[tmp]=0;
      flag_crossroad_2[tmp]=' ';
      sema_up(&intersection[tmp]);
   }
}

int position_acquire(int start, int dest, char id)
{
   int num=intersections_path[start][dest][0];
   int tmp=0;
   int check_acquire=1;
   for(int i=1; i<num+1; i++){
      tmp=intersections_path[start][dest][i];
      if (flag_crossroad[tmp]!=0){
         check_acquire=0;
         break;
      }
   }
   if(check_acquire==0){ //if there's intersection used by other thread(value is 1), then fail.
      return 0;
   }else{
      for(int i=1; i<num+1; i++){
         tmp=intersections_path[start][dest][i];
         sema_down(&intersection[tmp]);
         flag_crossroad[tmp]=1;
         flag_crossroad_2[tmp]=id;
         sema_up(&intersection[tmp]);
      }
      return 1;
   }

}
/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
   struct position pos_cur, pos_next;
   int check_valid=1;
   bool check_lock_sema=false;

   pos_next = vehicle_path[start][dest][step];
   pos_cur = vi->position;

   if (vi->state == VEHICLE_STATUS_RUNNING) {
      /* check termination */
      if (is_position_outside(pos_next)) {
         /* actual move */
         vi->position.row = vi->position.col = -1;
         /* release previous */
         lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
         return 0;
      }

      if (is_position_intersection(pos_next) && flag_crossroad_2[start]!=vi->id){
         //if try to go intersection, then check if it's valid.
         //(check if all intersection part that need to pass is valid)
         //if already in intersection and running, flag_crossroad_2[start]==vi->id, so pass this.
         check_valid=intersection_thread(start,dest,vi);
      }
   }

   if (check_valid==1){
   /* lock next position */
	   check_lock_sema=lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
      if(check_lock_sema==true){
         //if there's many threads, so have to wait state will be fail to lock acquire
         if (vi->state == VEHICLE_STATUS_READY) {
            /* start this vehicle */
            vi->state = VEHICLE_STATUS_RUNNING;
         } else {
            /* release current position */
            lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
         }
         /* update position */
         vi->position = pos_next;
         if(is_position_intersection(pos_cur) && flag_crossroad_2[start]==vi->id){
            //if it's end of intersection use, then have to release.
            struct position pos_next_next;
            pos_next_next=vehicle_path[start][dest][step+1];
            if(!is_position_intersection(pos_next_next)){
               do{
                  check_lock_sema=sema_try_down(intersection_out);
               }while(check_lock_sema==false);
               out_intersection[0][0]+=1;
               int tmp= out_intersection[0][0];
               out_intersection[tmp][0]=start;
               out_intersection[tmp][1]=dest;
               sema_up(intersection_out);
            }
         }
         return 1;
      }
   }
   return 2;
}

int intersection_thread(int start, int dest, struct vehicle_info *vi){
   bool check_sema=true;
   int check_acquire=0;
   do{
   check_sema=sema_try_down(intersection_whole);
   }while(check_sema==false);
   //if there's no one checking intersection part have to pass is valied or not
   check_acquire=position_acquire(start,dest,vi->id);
   sema_up(intersection_whole);
   if (check_acquire==1){
      return 1;
   } else{
      return 0;
   }
   
}

void init_on_mainthread(int thread_cnt){
   /* Called once before spawning threads */
   step=malloc(sizeof(struct lock)*1); //for unitstep, crossroad
   flag=malloc(sizeof(struct semaphore)*1); //for flag, current_thread_num
   threads=malloc(sizeof(struct semaphore)*26); //for each thread
   intersection=malloc(sizeof(struct semaphore)*4); //for each intersection
   intersection_whole=malloc(sizeof(struct semaphore)*1); //for whole intersection
   step_check=malloc(sizeof(struct semaphore)*1); //for start end of current step
   intersection_out=malloc(sizeof(struct semaphore)*1);
   lock_init(step);
   sema_init(flag,1);
   sema_init(intersection_whole,1);
   sema_init(step_check,1);
   sema_init(intersection_out,1);
   for(int i=0; i<4; i++){
      sema_init(&intersection[i],1);
      flag_crossroad[i]=0; 
      flag_crossroad_2[i]=' ';
   }
   for(int i=0; i<5; i++){
      for(int j=0; j<2; j++){
         out_intersection[i][j]=0;
      }
   }
   for(int i=0; i<26; i++){
      sema_init(&threads[i],1);
   }
   //intersection 0:(4,2),(4,3), 1:(4,4),(3,4), 2:(2,4),(2,3), 3:(2,2),(3,2)
   //flag_crossroad_1: if each intersection is used(1) or unused(0)
   //flag_crossroad_2: if each intersection is used(using thread's id) or unused(' ')
   __thread_cnt=thread_cnt;
   current_thread_num=0; //count current thread step is done and next step is exist
   flag_1=thread_cnt; //each threads minus if current step is done
   flag_2=0; //count ended thread
}

void check_unit_step(){
   
   sema_down(flag);
   flag_1=__thread_cnt - flag_2;
   current_thread_num=0;
   sema_up(flag);

   lock_acquire(step);
   crossroads_step+=1;
   unitstep_changed();
   lock_release(step);

   if (out_intersection[0][0]>0){
      sema_down(intersection_out);
      for(int i=1; i<out_intersection[0][0]+1;i++){
         position_initialize(out_intersection[i][0], out_intersection[i][1]);
      }
      out_intersection[0][0]=0;
      sema_up(intersection_out);
   }
   
   for(int i=0; i<26; i++){
      sema_up(&threads[i]);
   }

   
}
void vehicle_loop(void *_vi)
{
   int res;
   int start, dest, step;
   int thread;
   bool check_tmp=false;

   //current_thread_num+=1;
   struct vehicle_info *vi = _vi;

   thread = vi->id - 'a';
   start = vi->start - 'A';
   dest = vi->dest - 'A';

   vi->position.row = vi->position.col = -1;
   vi->state = VEHICLE_STATUS_READY;

   step = 0;
   while (1) {
      do{
         check_tmp=sema_try_down(&threads[thread]);
      }while(check_tmp==false);
      res = try_move(start, dest, step, vi);

      do{
         check_tmp=sema_try_down(flag);
      }while(check_tmp==false);
      flag_1-=1;
      if(res==0){
         flag_2+=1;
      }else{ //res=2 or 1 -> next step is exist
         current_thread_num+=1;
      }
      sema_up(flag);

      if (res == 1) {
         step++;
      }

      if (flag_1==0 && flag_2+current_thread_num==__thread_cnt){
         check_tmp=sema_try_down(step_check);
         if(check_tmp==true){
            check_unit_step();
            sema_up(step_check);
         }
      }

      /* termination condition. */ 
      if (res == 0) {
         break;
      }

   }   

   /* status transition must happen before sema_up */
   vi->state = VEHICLE_STATUS_FINISHED;
}