abstract class Hitbox {
    Vector2d centre;
    Hitbox(Vector2d centre){
        this.centre=centre;
    }
    abstract void changePosition(Vector2d newPosition);
    abstract void correctPosition(Vector2d distance);
}

class AABB extends Hitbox{
    Vector2d[] vertices;
    Vector2d firstAxis,secondAxis;

    /**
     * Constructor receives a centre and an array
     * of four vertices of an AABB
     */
    AABB(Vector2d centre,Vector2d[] vertices){
        super(centre);
        this.vertices=vertices;
        Vector2d firstSide=vertices[1].sub(vertices[0],new Vector2d());
        Vector2d secondSide=vertices[2].sub(vertices[1],new Vector2d());
        firstAxis=firstSide.normal().normalize();
        secondAxis=secondSide.normal().normalize();
    }

    @Override
    void changePosition(Vector2d newPosition) {
        Vector2d delta=newPosition.sub(centre,new Vector2d());
        centre.set(newPosition);
        for (int i=0;i<4;i++){
            vertices[i].add(delta);
        }
    }

    @Override
    void correctPosition(Vector2d distance) {
        changePosition(centre.add(distance,new Vector2d()));
    }
}

class CircleHitbox extends Hitbox{
    double radius;
    CircleHitbox(Vector2d centre,double radius){
        super(centre);
        this.radius=radius;
    }

    @Override
    void changePosition(Vector2d newPosition) {
        centre.set(newPosition);
    }

    @Override
    void correctPosition(Vector2d distance) {
        changePosition(centre.add(distance,new Vector2d()));
    }

}

class Pair<F,S>{
    F first;
    S second;
    Pair(F f,S s){
        first=f;
        second=s;
    }
}

class Intersection{
    private static Vector2d gramSchmidt(Vector2d axis,Vector2d vector){
        return vector.sub(axis.mul(vector.dotProduct(axis)/axis.dotProduct(axis)),new Vector2d());
    }
    
    private static Vector2d intersect(Vector2d A,Vector2d B,Vector2d C,Vector2d D){
        double x1=B.x,x0=A.x,y1=B.y,y0=A.y;
        double a=C.x,b=C.y,c=D.x,d=D.y;
        double l=c-a,q=d-b,n=x1-x0,m=y1-y0;
        double t=(n*(y0-b)+m*(a-x0))/(q*n-m*l);
        double x=a+t*l,y=b+t*q;
        return new Vector2d(x,y);
    }
    
    private static Vector2d determineLine(Vector2d orthogonal,Vector2d point){
        if (orthogonal.x==0){
            return new Vector2d(point.x,0);
        }
        if (orthogonal.y==0){
            return new Vector2d(0,point.y);
        }
        return new Vector2d(0,-orthogonal.y*point.x/orthogonal.x+point.y);
    }
    
    private static Vector2d calcLine(Vector2d a,Vector2d b,Vector2d point){
        Vector2d vector=b.sub(a,new Vector2d());
        Vector2d orthogonal=gramSchmidt(vector,point.sub(a,new Vector2d()));
        Vector2d inter=intersect(a,b,point,determineLine(orthogonal,point));
        double minX,minY,maxX,maxY;
        if (a.x<b.x){
            maxX=b.x;
            minX=a.x;
        }else {
            maxX=a.x;
            minX=b.x;
        }
        if (a.y<b.y){
            maxY=b.y;
            minY=a.y;
        }else {
            maxY=a.y;
            minY=b.y;
        }
        if (minX<=inter.x && inter.x<=maxX && inter.y<=maxY && inter.y>=minY){
            return inter;
        }else {
            Vector2d q=point.sub(a,new Vector2d());
            Vector2d l=point.sub(b,new Vector2d());
            if (q.lengthSquared()<l.lengthSquared()){
                return a;
            }else {
                return b;
            }
        }
    }
    
    static Vector2d findClosest(AABB box,Vector2d dot){
        Vector2d inter=calcLine(box.vertices[0],box.vertices[1],dot);
        Vector2d guid=inter.sub(dot,new Vector2d());
        int n=box.vertices.length;
        for (int i=1;i<n;i++) {
            Vector2d interNext = calcLine(box.vertices[i%n], box.vertices[(i + 1) % n], dot);
            Vector2d guidNext=interNext.sub(dot,new Vector2d());
            if (guid.lengthSquared()>guidNext.lengthSquared()){
                inter=interNext;
                guid=guidNext;
            }
        }
        return inter;
    }
}

class CollisionSolver{
    private static Pair<Double,Double> minMaxProjection(AABB object,Vector2d axis){
        double minPr,maxPr,projection;
        minPr=maxPr=object.vertices[0].vectorProjection(axis);
        for (int i=1;i<4;i++){
            projection=object.vertices[i].vectorProjection(axis);
            if (projection>maxPr){
                maxPr=projection;
            }else if (projection<minPr){
                minPr=projection;
            }
        }
        return new Pair<>(minPr,maxPr);
    }

    private static double overlap(Pair<Double,Double> pr1,Pair<Double,Double> pr2){
        if (pr1.second<pr2.second){
            return pr2.first-pr1.second;
        }else {
            return  pr1.first-pr2.second;
        }
    }

    private static Pair<Double,Vector2d> getMin(double dx,double dy,Vector2d firstAxis,Vector2d secondAxis){
        double depth;
        Vector2d axis;
        if (dx<dy){
            depth=dy;
            axis=new Vector2d(secondAxis);
        }else {
            depth=dx;
            axis=new Vector2d(firstAxis);
        }
        return new Pair<>(depth,axis);
    }

    private static Vector2d getCollision(AABB obj1,AABB obj2){
        Pair<Double,Double> thisPr1=minMaxProjection(obj1,obj1.firstAxis);
        Pair<Double,Double> objPr1=CollisionSolver.minMaxProjection(obj2,obj1.firstAxis);
        Pair<Double,Double> thisPr2=CollisionSolver.minMaxProjection(obj1,obj1.secondAxis);
        Pair<Double,Double> objPr2=CollisionSolver.minMaxProjection(obj2,obj1.secondAxis);

        Pair<Double,Double> secThisPr1=CollisionSolver.minMaxProjection(obj1,obj2.firstAxis);
        Pair<Double,Double> secObjPr1=CollisionSolver.minMaxProjection(obj2,obj2.firstAxis);
        Pair<Double,Double> secThisPr2=CollisionSolver.minMaxProjection(obj1,obj2.secondAxis);
        Pair<Double,Double> secObjPr2=CollisionSolver.minMaxProjection(obj2,obj2.secondAxis);

        double dx,dy,dX,dY;

        if ((dx=overlap(thisPr1,objPr1))<0 && (dy=overlap(thisPr2,objPr2))<0
                && (dX=overlap(secThisPr1,secObjPr1))<0 && (dY=overlap(secThisPr2,secObjPr2))<0){

            Pair<Double,Vector2d> thisMin=CollisionSolver.getMin(dx,dy,obj1.firstAxis,obj1.secondAxis);
            Pair<Double,Vector2d> objMin=CollisionSolver.getMin(dX,dY,obj2.firstAxis,obj2.secondAxis);
            Pair<Double,Vector2d> result=CollisionSolver.getMin(thisMin.first,objMin.first,thisMin.second,objMin.second);

            Vector2d centre_to_centre=obj1.centre.sub(obj2.centre,new Vector2d());
            if (centre_to_centre.vectorProjection(result.second)>0) result.second.mul(-1.0);
            return new Vector2d(result.second.mul(result.first));
        }
        return null;
    }

    private static Vector2d getCollision(AABB aabb,CircleHitbox circle){

        Vector2d intersectionPoint=Intersection.findClosest(aabb,circle.centre);
        Vector2d axis=intersectionPoint.sub(circle.centre,new Vector2d());
        double distance= circle.radius-axis.length();
        if (distance>0)
            return axis.normalize().mul(distance);
        return null;
    }

    private static Vector2d getCollision(CircleHitbox c1,CircleHitbox c2){
        Vector2d axis=c2.centre.sub(c1.centre,new Vector2d());
        double dist=axis.length();
        if (dist<c1.radius+c2.radius){
            return new Vector2d(axis.normalize().mul(c1.radius+c2.radius-dist));
        }
        return null;
    }
    static Vector2d getCollision(Hitbox h1,Hitbox h2){
        if (h1 instanceof AABB){
            if (h2 instanceof AABB) return getCollision((AABB)h1,(AABB)h2);
            if (h2 instanceof CircleHitbox) return getCollision((AABB)h1,(CircleHitbox)h2);
        }else if (h1 instanceof CircleHitbox){
            if (h2 instanceof CircleHitbox) return getCollision((CircleHitbox)h1,(CircleHitbox)h2);
            if (h2 instanceof AABB){
                Vector2d collision=getCollision((AABB)h2,(CircleHitbox)h1);
                if (collision != null) {
                    collision.mul(-1.0);
                }
                return collision;
            }
        }
        return null;
    }
}
