public class Vector2d {
    double x,y;
    Vector2d(){};

    Vector2d(double x,double y){
        set(x,y);
    }

    Vector2d(Vector2d vec){
        set(vec);
    }

    void set(double x,double y){
        this.x=x;
        this.y=y;
    }

    void set(Vector2d vec){
        set(vec.x,vec.y);
    }

    Vector2d add(double x,double y){
        this.x+=x;
        this.y+=y;
        return this;
    }

    Vector2d add(Vector2d vec){
        return add(vec.x,vec.y);
    }

    Vector2d add(double x,double y,Vector2d dest){
        dest.set(this.x,this.y);
        return dest.add(x,y);
    }

    Vector2d add(Vector2d vec,Vector2d dest){
        dest.set(x,y);
        return dest.add(vec);
    }

    Vector2d sub(double x,double y){
        return add(-x,-y);
    }

    Vector2d sub(Vector2d vec){
        return sub(vec.x,vec.y);
    }

    Vector2d sub(double x,double y,Vector2d dest){
        dest.set(this.x,this.y);
        return dest.sub(x,y);
    }

    Vector2d sub(Vector2d vec,Vector2d dest){
        dest.set(x,y);
        return dest.sub(vec);
    }

    Vector2d mul(Double number){
        x*=number;
        y*=number;
        return this;
    }

    Vector2d mul(Double number,Vector2d dest){
        dest.set(x,y);
        return dest.mul(number);
    }

    double lengthSquared(){
        return x*x+y*y;
    }

    double length(){
        return Math.sqrt(lengthSquared());
    }

    Vector2d normalize(){
        return new Vector2d(x/length(),y/length());
    }

    Vector2d normal(){
        return new Vector2d(-y,x);
    }

    double dotProduct(Vector2d vec){
        return x*vec.x+y*vec.y;
    }

    double vectorProjection(Vector2d vec){
        return dotProduct(vec.normalize());
    }
}
