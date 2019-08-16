abstract class Hitbox {
    Vector2d centre;
    Hitbox(Vector2d centre){
        this.centre=centre;
    }
    abstract Vector2d getCollision(Hitbox object);
    abstract void changePosition(Vector2d newPosition);
    abstract void correctPosition(Vector2d distance);
}

class AABB extends Hitbox{
    private Vector2d[] vertices;
    private Vector2d firstAxis,secondAxis;

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

    private Pair<Double,Double> minMaxProjection(AABB object,Vector2d axis){
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

    private double overlap(Pair<Double,Double> pr1,Pair<Double,Double> pr2){
        double min,max;
        if (pr1.second<pr2.second){
            max=pr1.second;
            min=pr2.first;
        }else {
            max=pr2.second;
            min=pr1.first;
        }
        return min-max;
    }

    private Pair<Double,Vector2d> getMin(double dx,double dy,Vector2d firstAxis,Vector2d secondAxis){
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

    /**
     * Returns vector every point of the polygon
     * will have been translated along
     */
    @Override
    Vector2d getCollision(Hitbox object) {
        if (object instanceof AABB){
            AABB obj=(AABB)object;
            Pair<Double,Double> thisPr1=minMaxProjection(this,firstAxis);
            Pair<Double,Double> objPr1=minMaxProjection(obj,firstAxis);
            Pair<Double,Double> thisPr2=minMaxProjection(this,secondAxis);
            Pair<Double,Double> objPr2=minMaxProjection(obj,secondAxis);

            Pair<Double,Double> secThisPr1=minMaxProjection(this,obj.firstAxis);
            Pair<Double,Double> secObjPr1=minMaxProjection(obj,obj.firstAxis);
            Pair<Double,Double> secThisPr2=minMaxProjection(this,obj.secondAxis);
            Pair<Double,Double> secObjPr2=minMaxProjection(obj,obj.secondAxis);

            double dx,dy,dX,dY;

            if ((dx=overlap(thisPr1,objPr1))<0 && (dy=overlap(thisPr2,objPr2))<0
                    && (dX=overlap(secThisPr1,secObjPr1))<0 && (dY=overlap(secThisPr2,secObjPr2))<0){

                Pair<Double,Vector2d> thisMin=getMin(dx,dy,firstAxis,secondAxis);
                Pair<Double,Vector2d> objMin=getMin(dX,dY,obj.firstAxis,obj.secondAxis);
                Pair<Double,Vector2d> result=getMin(thisMin.first,objMin.first,thisMin.second,objMin.second);

                Vector2d centre_to_centre=centre.sub(obj.centre,new Vector2d());
                if (centre_to_centre.vectorProjection(result.second)>0) result.second.mul(-1.0);
                return new Vector2d(result.second.mul(result.first));
            }
        }
        if (object instanceof CircleHitbox){
            CircleHitbox obj=(CircleHitbox)object;

            Vector2d axis=obj.centre.sub(centre,new Vector2d());
            Pair<Double,Double> thisPr=minMaxProjection(this,axis);

            double centreProjection=obj.centre.vectorProjection(axis);
            Pair<Double,Double> struct=new Pair<>(centreProjection-obj.radius,centreProjection+obj.radius);

            double depth;
            if ((depth=overlap(thisPr,struct))<0){
                return new Vector2d(axis.normalize().mul(depth));
            }
        }
        return null;
    }

	/**
	 * Changes the postion of the hitbox
	 */
    @Override
    void changePosition(Vector2d newPosition) {
        Vector2d delta=newPosition.sub(centre,new Vector2d());
        centre.set(newPosition);
        for (int i=0;i<4;i++){
            vertices[i].add(delta);
        }
    }
    
	/**
	 * Must be called if a collision occurred
	 */
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

    /**
     * Returns vector every point of the polygon
     * will have been translated along
     */
    @Override
    Vector2d getCollision(Hitbox object) {
        if (object instanceof AABB){
            AABB obj=(AABB)object;
            Vector2d depth=obj.getCollision(this);
            if (depth!=null){
                return depth.mul(-1.0);
            }
        }else if (object instanceof CircleHitbox){
            CircleHitbox obj=(CircleHitbox) object;

            Vector2d axis=obj.centre.sub(centre,new Vector2d());
            double dist=axis.length();
            if (dist<radius+obj.radius){
                return new Vector2d(axis.normalize().mul(radius+obj.radius-dist));
            }
        }
        return null;
    }
	
	/**
	 * Changes the postion of the hitbox
	 */
    @Override
    void changePosition(Vector2d newPosition) {
        centre.set(newPosition);
    }
	
	/**
	 * Must be called if a collision occurred
	 */
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
