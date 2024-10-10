// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"
void followBoundary(){

    if (leftFollow) { //đang né vật cản
      setSpeed(0.045,0.16);
      if ((rightDistance>0)&&(rightDistance < 15)){// nếu đang né mà thấy sắp đụng phải vật cản 
      setSpeed(0.16,0.04); //thì rẽ qua trái để đi xa khỏi vật cản
      } else if ((rightDistance>0)&&(rightDistance > 15)){// nếu đang né mà thấy sắp quá xa vật cản 
      setSpeed(0.045,0.16); //thì rẽ qua phải để đi quanh khỏi vật cản
      } 
      // setSpeed(v-rightDistance*0.003,v);
    }else if (rightFollow) {//đang né vật cản
      // setSpeed(v,v-rightDistance*0.003);
        setSpeed(0.16,0.045);
    if ((leftDistance>0)&&(leftDistance < 15)){ 
      setSpeed(0.04,0.16);
      } else if ((rightDistance>0)&&(leftDistance > 15)){// nếu đang né mà thấy sắp quá xa vật cản 
      setSpeed(0.16,0.045); //thì rẽ qua trái để đi quanh khỏi vật cản
      } 
    }
    else {//mới hay tin là phía trước có vật cản
    if (!isLeftObstacle) { //nếu bên trái trống
      rotateLeft(); //quẹo trái
      setSpeed(0.1,0.15);//đi vòng tròn thuận chiều kim đồng hồ để ôm quanh vật cản
      leftFollow =1; // lưu biến để lần tới trong vòng lặp robot sẽ biết mình đang né vật cản
    } else if (!isRightObstacle) { //tương tự
      rotateRight();
      setSpeed(0.15,0.1);
      rightFollow =1;
    }
    delay(50); 
    }
}
