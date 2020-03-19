import java.util.HashSet;

abstract class Hitbox {
    Vector2d centre;
    Hitbox(Vector2d centre){
        this.centre=centre;
    }
    abstract void changePosition(Vector2d newPosition);
    abstract void correctPosition(Vector2d distance);
}

class PolygonHitbox extends Hitbox{
    Vector2d[] vertices;
    HashSet<Vector2d> axes; // Collinear normals are considered equal
    PolygonHitbox(Vector2d centre,Vector2d[] vertices) {
        super(centre);
        this.vertices=vertices;
        axes=new HashSet<>();
        for (int i=vertices.length-1;i>0;i--){
            axes.add(vertices[i].sub(vertices[i-1],new Vector2d()).normal().normalize());
        }
        axes.add(vertices[0].sub(vertices[vertices.length-1],new Vector2d()).normal().normalize());
    }

    
    void changePosition(Vector2d newPosition) {
        Vector2d delta=newPosition.sub(centre,new Vector2d());
        centre.set(newPosition);
        for (Vector2d vertex : vertices) {
            vertex.add(delta);
        }
    }

    
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

    
    void changePosition(Vector2d newPosition) {
        centre.set(newPosition);
    }

    
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

    
    public String toString() {
        return "Pair{" +
                "first=" + first +
                ", second=" + second +
                '}';
    }
}

class CollisionSolver{
    private static Vector2d getCollision(PolygonHitbox ph1,PolygonHitbox ph2){
        Pair<Double,Double> pr1,pr2;
        double delta;
        Pair<Double,Vector2d> result=new Pair<>(10.0,new Vector2d()); // first value should be big enough
        for (Vector2d axis : ph1.axes){ // finding an axis, projection on which is minimal
            pr1= Calc.minMaxProjection(ph1,axis);
            pr2= Calc.minMaxProjection(ph2,axis);
            delta= Calc.overlap(pr1,pr2);
            if (delta>=0){
                return null;
            }
            if (-delta<result.first){
                result.first=-delta;
                result.second=axis;
            }
        }

        for (Vector2d axis : ph2.axes){ // again
            pr1=Calc.minMaxProjection(ph1,axis);
            pr2= Calc.minMaxProjection(ph2,axis);
            delta= Calc.overlap(pr1,pr2);
            if (delta>=0){
                return null;
            }
            if (-delta<result.first){
                result.first=-delta;
                result.second=axis;
            }
        }

        //calculating this vector helps us to understand the mutual arrangement of the objects
        Vector2d centre_to_centre=ph1.centre.sub(ph2.centre,new Vector2d());

        if (centre_to_centre.vectorProjection(result.second)<0) result.first*=-1;

        return result.second.mul(result.first,new Vector2d());

    }

    private static Vector2d getCollision(PolygonHitbox aabb,CircleHitbox circle){
        Vector2d intersectionPoint= Calc.findClosest(aabb,circle.centre);
        Vector2d axis=intersectionPoint.sub(circle.centre,new Vector2d());
        double distance= circle.radius-axis.length();
        if (distance>0){
            return axis.normalize().mul(distance);
        }
        return null;
    }

    private static Vector2d getCollision(CircleHitbox circle,PolygonHitbox ph){
        Vector2d c=getCollision(ph,circle);
        if (c!=null) c.mul(-1.0);
        return c;
    }

    private static Vector2d getCollision(CircleHitbox c1,CircleHitbox c2){
        Vector2d axis=c2.centre.sub(c1.centre,new Vector2d());
        double dist=axis.length();
        if (dist<c1.radius+c2.radius){
            return new Vector2d(axis.normalize().mul(c1.radius+c2.radius-dist));
        }
        return null;
    }


    private static class Calc {
        private static Vector2d gramSchmidt(Vector2d axis,Vector2d vector){
            return vector.sub(axis.mul(vector.dotProduct(axis)/axis.dotProduct(axis)),new Vector2d());
        }

        /**
         * Calculates intersection of two lines. Each line is represented by two dots.
         * @param A Vector2d
         * @param B Vector2d
         * @param C Vector2d
         * @param D Vector2d
         * @return Vector2d
         */

        private static Vector2d intersect(Vector2d A,Vector2d B,Vector2d C,Vector2d D){
            double x1=B.x,x0=A.x,y1=B.y,y0=A.y;
            double a=C.x,b=C.y,c=D.x,d=D.y;
            double l=c-a,q=d-b,n=x1-x0,m=y1-y0;
            double t=(n*(y0-b)+m*(a-x0))/(q*n-m*l);
            double x=a+t*l,y=b+t*q;
            return new Vector2d(x,y);
        }


        /**
         * Returns another point belongs to the line
         * @param guidVector Vector2d
         * @param point Vector2d
         * @return
         */

        private static Vector2d determineLine(Vector2d guidVector,Vector2d point){
            if (guidVector.x==0){
                return new Vector2d(point.x,0);
            }
            if (guidVector.y==0){
                return new Vector2d(0,point.y);
            }
            return new Vector2d(0,-guidVector.y*point.x/guidVector.x+point.y);
        }

        /**
         * Calculates the closest distance between point
         * and section represented by two dots
         * @param dot1 Vector2d
         * @param dot2 Vector2d
         * @param point Vector2d
         * @return
         */

        private static Vector2d calcLine(Vector2d dot1,Vector2d dot2,Vector2d point){
            Vector2d vector=dot2.sub(dot1,new Vector2d());
            Vector2d orthogonal=gramSchmidt(vector,point.sub(dot1,new Vector2d()));
            Vector2d inter=intersect(dot1,dot2,point,determineLine(orthogonal,point));
            double minX,minY,maxX,maxY;
            if (dot1.x<dot2.x){
                maxX=dot2.x;
                minX=dot1.x;
            }else {
                maxX=dot1.x;
                minX=dot2.x;
            }
            if (dot1.y<dot2.y){
                maxY=dot2.y;
                minY=dot1.y;
            }else {
                maxY=dot1.y;
                minY=dot2.y;
            }
            if (minX<=inter.x && inter.x<=maxX && inter.y<=maxY && inter.y>=minY){
                return inter;
            }else {
                Vector2d q=point.sub(dot1,new Vector2d());
                Vector2d l=point.sub(dot2,new Vector2d());
                if (q.lengthSquared()<l.lengthSquared()){
                    return dot1;
                }else {
                    return dot2;
                }
            }
        }

        /**
         * Returns a point on the hitbox's edge closest to the given one
         * @param box PolygonHitbox
         * @param dot Vector2d
         * @return Vector2d
         */

        static Vector2d findClosest(PolygonHitbox box,Vector2d dot){
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

        /**
         * Returns min and max projections on the given axis
         * @param object PolygonHitbox
         * @param axis Vector2d
         * @return Pair
         */

        private static Pair<Double,Double> minMaxProjection(PolygonHitbox object, Vector2d axis){
            double minPr,maxPr,projection;
            minPr=maxPr=object.vertices[0].vectorProjection(axis);
            for (int i=1;i<object.vertices.length;i++){
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
    }
}
