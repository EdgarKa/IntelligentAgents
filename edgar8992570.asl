// Agent ed in project miningAgents.mas2j D



/* Initial beliefs and rules */

next(X0,Y0,up,X,Y) :- X=X0 & Y = Y0-1.
next(X0,Y0,down,X,Y) :- X=X0 & Y = Y0+1.
next(X0,Y0,left,X,Y) :- X=X0-1 & Y = Y0.
next(X0,Y0,right,X,Y) :- X=X0+1 & Y = Y0.

inGrid(X,Y) :- gsize(_, Gx, Gy) & X >= 0 & X < Gx & Y >= 0 & Y < Gy.

nextAvailable(X0,Y0,D,X,Y) :- next(X0,Y0,D,X,Y) & inGrid(X,Y) & 
	not cell(X,Y,obstacle) & not cell(X,Y,ally) & not visited(X,Y).


//-------------------------------------------------------------

getAbsoluteValue(X0, X, Result) :- X0>X & Result = X0-X.
getAbsoluteValue(X0, X, Result) :- X0 <= X & Result = X-X0.

evaluate([],[]) :- true.

evaluate([cell(Xgold,Ygold,gold) | Rest], [distance(UtilityOfARun,cell(Xgold,Ygold,gold)) | RestOfNewList]) :-
	evaluate(distance(UtilityOfARun,cell(Xgold,Ygold,gold))) & evaluate(Rest,RestOfNewList).

evaluate(distance(UtilityOfARun,cell(Xgold,Ygold,gold))) :-
	pos(X0,Y0) & getAbsoluteValue(X0,Xgold,ResultX) & 
	getAbsoluteValue(Y0,Ygold,ResultY) & UtilityOfARun = ResultX + ResultY.

//-------------------------------------------------------------

opposite(up,down).
opposite(down,up).
opposite(left,right).
opposite(right,left).

randomDirection(R,D) :- R <= 0.25 & D = up.
randomDirection(R,D) :- R <= 0.5 & D = down.
randomDirection(R,D) :- R <= 0.75 & D = left.
randomDirection(R,D) :- R <= 1.0 & D = right.

randomNumber(R, Max, N) :- N = ((Max-1)*R)div 1.

/* Initial goals */

!clean.

/* Plans */

+!clean : not cell(X,Y,gold)[source(self)] 
	<-
	!tryRandomCell;
	!clean.
	
+!clean : cell(Xgold,Ygold,gold)[source(self)] 
	<-
	!findNearestGold(X,Y);
	?pos(Xcur,Ycur);
	!tryGoTo(X,Y); 
	!tryPick;
	.abolish(foundGold(X,Y));
	?depot(_,Xd,Yd); 
	!tryGoTo(Xd,Yd);
	!tryDrop;
	-busy;
	!clean.

// findNearestGold is the plan, that finds the nearest gold by taking all of
// the beliefs about existing gold and finding the one closest to the agent
+!findNearestGold(X,Y): 
	.findall(cell(X,Y,gold),cell(X,Y,gold), L) & 
	evaluate(L,NL) & .min(NL, distance(UtilityOfARun,cell(Xgold,Ygold,gold))) & 
	X = Xgold & Y = Ygold 
	<- 
	true.

// tryRandomCell generates random numbers within a grid and sets them as a
// destination
+!tryRandomCell : 
	.random(RX) & .random(RY) & gsize(SimID,W,H) & randomNumber(RX, W, Xdest) & 
	randomNumber(RY, H, Ydest) &	not cell(Xdest,Ydest,obstacle) //& not cleaned 
	<-
	!tryGoTo(Xdest,Ydest).

+!tryRandomCell: true 
	<-
	!tryRandomCell.

// to ensure the gold will be picked up
+!tryPick : pos(X,Y) & not cell(X,Y,gold)[source(percept)] 
	<-
	true.

+!tryPick : pos(X,Y) & cell(X,Y,gold)[source(percept)] & not carrying_gold 
	<-
	do(pick); 
	!tryPick.

+!tryPick : pos(X,Y) & cell(X,Y,gold)[source(percept)] & carrying_gold 
	<-
	true.	
	
// this plan is to insure the gold will be dropped
+!tryDrop : not carrying_gold 
	<-
	true.
+!tryDrop : carrying_gold
	<-
	do(drop);
	!tryDrop.
	
+!tryGoTo(X,Y) : true 
	<-	
	.abolish(visited(_,_)); 
	.abolish(preMove(_,_,_));	
	!goTo(X,Y).
	
+!goTo(X,Y) : pos(X,Y) <- true. // if it reached destination

+!goTo(X,Y) : cell(X,Y,obstacle) <- true. // is that one necessary?

+!goTo(Xdest,Ydest) : pos(Xcur,Ycur) & nextAvailable(Xcur,Ycur,D,Xcell,Ycell) 
	<-
	+visited(Xcur,Ycur);
	!goTowards(Xdest,Ydest);
	+preMove(Xcell,Ycell,D); 
	!goTo(Xdest,Ydest).

+!goTo(Xdest,Ydest) : pos(Xcur,Ycur) & preMove(Xcur,Ycur,D1) & 
	opposite(D1,D2) & nextAvailable(Xcur,Ycur,D2,X,Y) 
	<-
	+visited(Xcur,Ycur);
	do(D2); 	
	!goTo(Xdest,Ydest).
	
+!goTo(X,Y) <- true.

//checks for gold and obstacles in 4 directions from an agent
+!update : pos(Xcur,Ycur) <-
	!check(cell(Xcur,Ycur,gold));
	!check(cell(Xcur-1,Ycur,gold));
	!check(cell(Xcur+1,Ycur,gold));
	!check(cell(Xcur,Ycur-1,gold));
	!check(cell(Xcur,Ycur+1,gold));
	!check(cell(Xcur-1,Ycur,obstacle));
	!check(cell(Xcur+1,Ycur,obstacle));
	!check(cell(Xcur,Ycur-1,obstacle));
	!check(cell(Xcur,Ycur+1,obstacle)).
	
+!check(cell(X,Y,obstacle)) : cell(X,Y,obstacle) 
	<-
	+cell(X,Y,obstacle);
	+cleaned(X,Y).
	
+!check(cell(X,Y,gold)) : cell(X,Y,gold)[source(percept)] 
	<-
	+cell(X,Y,gold).
	//.broadcast(tell, cell(X,Y,gold)).
	
+!check(cell(X,Y,gold)) : not cell(X,Y,gold)[source(percept)] 
	<-
	-cell(X,Y,gold);
	+cleaned(X,Y).
	
+!check(_) <- true. // stops, if there's no gold

//checks for available directions towards the target
 // priority for up and left (top left)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
// priority for left (left)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
// priority for down and left (bottom left)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X<Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
// priority for up and right (top right)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
// priority for right (right)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y==Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
// priority for down and right (bottom right)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X>Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
// priority for up (top)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y<Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
// priority for down (bottom)
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,down,X1,Y1) <- !direction(X,Y,down).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,left,X1,Y1) <- !direction(X,Y,left).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,right,X1,Y1) <- !direction(X,Y,right).
+!goTowards(X,Y) : pos(Xcur,Ycur) & X==Xcur & Y>Ycur & nextAvailable(Xcur,Ycur,up,X1,Y1) <- !direction(X,Y,up).

+!direction(X,Y,D) : pos(Xcur,Ycur) & nextAvailable(Xcur,Ycur,D,X1,Y1) 
	<-
	+visited(Xcur,Ycur);
	!update;
	+cleaned(Xcur,Ycur);
	.broadcast(tell,cleaned(Xcur,Ycur));
	do(D);
	!update;
	+preMove(X1,Y1,D).
	
	
+cell(Xgold,Ygold,gold)[source(percept)] : 
	not cell(Xgold,Ygold,gold)[source(self)] & not carrying_gold
	<-
	.broadcast(tell,foundGold(Xgold,Ygold));
	+cell(Xgold,Ygold,gold);
	.drop_all_intentions;
	!clean.
	
+cell(Xgold,Ygold,gold)[source(percept)] : 
	not cell(Xgold,Ygold,gold)[source(self)] & carrying_gold
	<-
	.broadcast(tell,foundGold(Xgold,Ygold));
	+cell(Xgold,Ygold,gold).
	
+cell(Xgold,Ygold,gold)[source(percept)]: cell(Xgold,Ygold,gold)[source(self)]
	<-
	true.
	
+foundGold(Xgold,Ygold): carrying_gold 
	<-
	+cell(Xgold,Ygold,gold);
	.broadcast(tell,cell(Xgold,Ygold,gold)).
	
+foundGold(Xgold,Ygold): not carrying_gold 
	<-
	+cell(Xgold,Ygold,gold);
	.drop_all_intentions;
	!clean.
	
+carrying_gold: pos(X0,Y0) 
	<-
	.broadcast(tell,claimed(X0,Y0));
	+cleaned(X0,Y0);
	-cell(X0,Y0, gold).

+claimed(Xgold,Ygold): true 
	<-
	.drop_all_intentions;
	-cell(Xgold,Ygold,gold);
	!clean.
	
