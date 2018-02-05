package com.base.engine.physics;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import org.joml.Vector2f;
import org.joml.Vector3f;

public class CollisionDetector
{    
    private enum Type
    {
        FACEA, FACEB, EDGE;
    }
    
    public Manifold separatingAxisTheorem(Body bodyA, Body bodyB)
    {
        Vector3f offset = new Vector3f(bodyB.getPosition()).sub(bodyA.getPosition());
        Manifold results = new Manifold(bodyA, bodyB);
        
        for(int i = 0; i < bodyA.getFaceCount(); i++)
        {
            Vector3f axis = new Vector3f(bodyA.getOrientatedFaceNormal(i));
            
            Vector3f planePoint = bodyA.getSupport(axis);
            Plane plane = new Plane(axis, planePoint);
            Vector3f support = bodyB.getSupport(new Vector3f(plane.getNormal()).negate());
            float distance = plane.distanceToPoint(support);
            if(distance > 0)
            {
                return results;
            }
            if(distance > results.getPenetration())
            {
                results.setPenetration(distance);
                results.setType(Type.FACEA.ordinal());
                results.setEnterNormal(axis);
                results.setReferenceFace(bodyA.getFace(i));
                results.setRefFace(i);
            }
        }
        
        for(int i = 0; i < bodyB.getFaceCount(); i++)
        {
            Vector3f axis = new Vector3f(bodyB.getOrientatedFaceNormal(i));
            
            Vector3f planePoint = bodyB.getSupport(axis);
            Plane plane = new Plane(axis, planePoint);
            Vector3f support = bodyA.getSupport(new Vector3f(plane.getNormal()).negate());
            float distance = plane.distanceToPoint(support);
            if(distance > 0)
            {
                return results;
            }
            if(distance > results.getPenetration())
            {
                results.setPenetration(distance);
                results.setType(Type.FACEB.ordinal());
                results.setEnterNormal(axis);
                results.setReferenceFace(bodyB.getFace(i));
                results.setRefFace(i);
            }
        }
        
        ArrayList<Edge> edgesA = bodyA.getEdges();
        ArrayList<Edge> edgesB = bodyB.getEdges();
        for(int i = 0; i < edgesA.size(); i++)
        {
            Vector3f a = bodyA.getOrientatedFaceNormal(edgesA.get(i).getFaceA());
            Vector3f b = bodyA.getOrientatedFaceNormal(edgesA.get(i).getFaceB());
            for(int j = 0; j < edgesB.size(); j++)
            {                
                Vector3f c = bodyB.getOrientatedFaceNormal(edgesB.get(j).getFaceA());
                c.negate();
                Vector3f d = bodyB.getOrientatedFaceNormal(edgesB.get(j).getFaceB());
                d.negate();
                
                Vector3f ba = new Vector3f(bodyA.getTranslatedEdgeDirection(i));
                Vector3f dc = new Vector3f(bodyB.getTranslatedEdgeDirection(j));
                float cba = c.dot(ba);
                float dba = d.dot(ba);
                float adc = a.dot(dc);
                float bdc = b.dot(dc);
                if(cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0)
                {                    
                    Vector3f axis = new Vector3f(bodyA.getTranslatedEdgeDirection(i)).cross(bodyB.getTranslatedEdgeDirection(j));
                    if(axis.length() < 0.00000001f)
                    {
                        continue;
                    }
                    axis.normalize();
                    
                    Vector3f planePoint = bodyA.getSupport(axis);
                    
                    Plane plane = new Plane(axis, planePoint);
                    Vector3f support = bodyB.getSupport(new Vector3f(plane.getNormal()).negate());
                    float distance = plane.distanceToPoint(support);
                    if(distance > 0)
                    {
                        return results;
                    }
                    if(distance > results.getPenetration())
                    {
                        results.setPenetration(distance);
                        results.setType(Type.EDGE.ordinal());
                        results.setEnterNormal(axis);
                        results.setEdgeA(edgesA.get(i).getTranslatedDirection(bodyA));
                        results.setEdgeB(edgesB.get(j).getTranslatedDirection(bodyB));
                        results.setSupportA(planePoint);
                        results.setSupportB(support);
                    }
                }
            }
        }
        
        Body reference, incident;
        if(results.getType() == Type.FACEA.ordinal())
        {
            reference = bodyA;
            incident = bodyB;
            getFaceContactPoints(reference, incident, results);
        }
        else if (results.getType() == Type.FACEB.ordinal())
        {
            reference = bodyB;
            incident = bodyA;
            getFaceContactPoints(reference, incident, results);
        }
        else
        {
            reference = bodyA;
            incident = bodyB;
            getEdgeContactPoint(reference, incident, results);
        }
        
        if(results.getEnterNormal().dot(offset) > 0)
        {
            results.getEnterNormal().negate();
        }
        
        if(results.getEnterTime() < 0 || results.getEnterTime() > 1)
        {
            results.setOverlapped();
        }
        else
        {
            results.setCollided();
        }
        return results;
    }
    
    private void getFaceContactPoints(Body reference, Body incident, Manifold results)
    {
        Vector3f referenceAxis = new Vector3f(results.getEnterNormal());
        
        int incidentFace = -1;
        float minDot = Float.MAX_VALUE;
        for(int i = 0; i < incident.getFaceCount(); i++)
        {
            float dot = referenceAxis.dot(incident.getOrientatedFaceNormal(i));
            if(dot < minDot)
            {
                minDot = dot;
                incidentFace = i;
            }
        }
        
        Set<Plane> planes = getSidePlanes(reference, results.getRefFace());
        
        ArrayList<Vector3f> faceVertices = getVerticesOfFace(incident, incidentFace);
        
        Iterator<Plane> iterator = planes.iterator();
        while(iterator.hasNext())
        {
            faceVertices = clipFace(iterator.next(), faceVertices);
        }
        faceVertices = getDeepestPoints(reference, results.getRefFace(), faceVertices);
        
        for(Vector3f e : faceVertices)
        {
            results.addContactPoint(e);
        }
    }
    
    private void getEdgeContactPoint(Body reference, Body incident, Manifold results)
    {        
        //get reference and incident edges that the contact point is going to reside near
        Vector3f referenceEdge = new Vector3f(results.getEdgeA());
        Vector3f incidentEdge = new Vector3f(results.getEdgeB());
        
        //get the edge points that are farthest along the collision normal so that we can find the true reference and incident edges which contain these points and the point of contact
        Vector3f referenceEdgePoint = new Vector3f(results.getSupportA());
        Vector3f incidentEdgePoint = new Vector3f(results.getSupportB());
        
        //get the actual edges that the contact point will reside on
        Edge edgeA = null, edgeB = null;
        float shortestA = Float.MAX_VALUE, shortestB = Float.MAX_VALUE;
        for(int i = 0; i < reference.getFaceCount(); i++)
        {
            for(int j = 0; j < reference.getEdgesOfFace(i).size(); j++)
            {
                Edge edge = reference.getEdge(reference.getFace(i).getEdgeIndices().get(j));
                Edge real;
                if(edge.getFaceB() == i)
                {
                    Vector3f transformedA = new Vector3f(edge.getPointB());
                    reference.translateVertex(transformedA);
                    Vector3f transformedB = new Vector3f(edge.getPointA());
                    reference.translateVertex(transformedB);
                    real = new Edge(transformedA, transformedB);
                }
                else
                {
                    Vector3f transformedA = new Vector3f(edge.getPointA());
                    reference.translateVertex(transformedA);
                    Vector3f transformedB = new Vector3f(edge.getPointB());
                    reference.translateVertex(transformedB);
                    real = new Edge(transformedA, transformedB);
                }
                
                if(real.containsPoint(referenceEdgePoint))
                {
                    float distance = new Vector3f(real.getDirection()).normalize().distance(referenceEdge);
                    if(distance < shortestA)
                    {
                        edgeA = real;
                        shortestA = distance;
                    }
                }
            }
        }
        
        for(int i = 0; i < incident.getFaceCount(); i++)
        {
            for(int j = 0; j < incident.getEdgesOfFace(i).size(); j++)
            {
                Edge edge = incident.getEdge(incident.getFace(i).getEdgeIndices().get(j));
                Edge real;
                if(edge.getFaceB() == i)
                {
                    Vector3f transformedA = new Vector3f(edge.getPointA());
                    incident.translateVertex(transformedA);
                    Vector3f transformedB = new Vector3f(edge.getPointB());
                    incident.translateVertex(transformedB);
                    real = new Edge(transformedA, transformedB);
                }
                else
                {
                    Vector3f transformedA = new Vector3f(edge.getPointB());
                    incident.translateVertex(transformedA);
                    Vector3f transformedB = new Vector3f(edge.getPointA());
                    incident.translateVertex(transformedB);
                    real = new Edge(transformedA, transformedB);
                }
                
                if(real.containsPoint(incidentEdgePoint))
                {
                    float distance = new Vector3f(real.getDirection()).normalize().distance(incidentEdge);
                    if(distance < shortestB)
                    {
                        edgeB = real;
                        shortestB = distance;
                    }
                }
            }
        }
                     
        //get the contact point via the closest points between the reference and incident edges
        Vector3f a1 = new Vector3f(edgeA.getPointA());
        Vector3f a2 = new Vector3f(edgeA.getPointB());
        Vector3f b1 = new Vector3f(edgeB.getPointA());
        Vector3f b2 = new Vector3f(edgeB.getPointB());
        Vector3f u = new Vector3f(a2).sub(a1);
        Vector3f v = new Vector3f(b2).sub(b1);
        Vector3f w = new Vector3f(a1).sub(b1);
        float a = u.dot(u);
        float b = u.dot(v);
        float c = v.dot(v);
        float d = u.dot(w);
        float e = v.dot(w);
        float D = a * c - b * b;
        
        float sN = 0, tN = 0, sD = D, tD = D;
        if(D < 0.0001f)
        {
            sN = 0;
            sD = 1;
            tN = e;
            tD = c;
        }
        else
        {
            sN = b * e - c * d;
            tN = a * e - b * d;
            if(sN < 0)
            {
                sN = 0;
                tN = e;
                tD = c;
            }
            else if(sN > sD)
            {
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }
        if(tN < 0)
        {
            tN = 0;
            if(-d < 0)
            {
                sN = 0;
            }
            else if(-d > a)
            {
                sN = sD;
            }
            else
            {
                sN = -d;
                sD = a;
            }
        }
        else if(tN > tD)
        {
            tN = tD;
            
            if((-d + b) < 0)
            {
                sN = 0;
            }
            else if((-d + b) > a)
            {
                sN = sD;
            }
            else
            {
                sN = (-d + b);
                sD = a;
            }
        }
        
        float sc = Math.abs(sN) < 0.0001f ? 0 : sN / sD;
        float tc = Math.abs(tN) < 0.0001f ? 0 : tN / tD;
        Vector3f o = new Vector3f(u).mul(sc);
        Vector3f n = new Vector3f(v).mul(tc);
        Vector3f dP = new Vector3f(w).add(new Vector3f(u).mul(sc)).sub(new Vector3f(v).mul(tc));
        o.add(a1);
        n.add(b1);
        
        Vector3f contact = new Vector3f(o).add(n);
        contact.div(2);
        results.addContactPoint(contact);
        //contact point's normal should be the cross product of both edges involed in the collision
    
    }
    
    private Set<Plane> getSidePlanes(Body object, int face)
    {
        Set<Plane> planes = new HashSet<>();
        
        //get each edge on the face
        for(int j = 0; j < object.getEdgesOfFace(face).size(); j++)
        {
            Edge edge = object.getEdgesOfFace(face).get(j);

            int other = edge.getOtherFaceOnEdge(face);
            Vector3f normal = object.getOrientatedFaceNormal(other);

            Vector3f point = new Vector3f(edge.getPointB());
            object.translateVertex(point);
            Plane plane = new Plane(normal, point);
            planes.add(plane);
        }
        return planes;
    }
    
    private ArrayList<Vector3f> getVerticesOfFace(Body object, int face)
    {
        ArrayList<Vector3f> vertices = object.getTransformedVerticesOfFace(object.getFace(face));
        return vertices;
    }
    
    private Vector3f getIntersectionPoint(Plane plane, Vector3f pointA, Vector3f pointB)
    {
        float distanceA = plane.distanceToPoint(pointA);
        float distanceB = plane.distanceToPoint(pointB);
        Vector3f direction = new Vector3f(pointB).sub(pointA);
        float alpha = distanceA / (distanceA - distanceB);
        Vector3f intersection = new Vector3f(direction).mul(alpha);
        return new Vector3f(pointA).add(intersection);
    }
    
    private ArrayList<Vector3f> clipFace(Plane plane, ArrayList<Vector3f> input)
    {
        ArrayList<Vector3f> safe = new ArrayList<>();
        int j = 1;
        for(int i = 0; i < input.size(); i++)
        {
            if(j >= input.size())
            {
                j = 0;
            }
            Vector3f pointA = new Vector3f(input.get(i));
            Vector3f pointB = new Vector3f(input.get(j));
            boolean contactedA = plane.isPointBehindPlane(pointA);
            boolean contactedB = plane.isPointBehindPlane(pointB);
            if(contactedA && contactedB)
            {
                if(!safe.contains(pointA))
                {
                    safe.add(pointA);
                }
                if(!safe.contains(pointB))
                {
                    safe.add(pointB);
                }
            }
            else if(!contactedA && contactedB)
            {
                Vector3f newPointA = getIntersectionPoint(plane, pointA, pointB);
                if(!safe.contains(newPointA))
                {
                    safe.add(newPointA);
                }
                if(!safe.contains(pointB))
                {
                    safe.add(pointB);
                }
            }
            else if(contactedA && !contactedB)
            {
                Vector3f newPointB = getIntersectionPoint(plane, pointA, pointB);
                if(!safe.contains(pointA))
                {
                    safe.add(pointA);
                }
                if(!safe.contains(newPointB))
                {
                    safe.add(newPointB);
                }
            }
            j++;
        }
        return safe;
    }
    
    private ArrayList<Vector3f> getDeepestPoints(Body object, int face, ArrayList<Vector3f> clipPoints)
    {
        ArrayList<Vector3f> safe = new ArrayList<>();
        
        Vector3f pointOnPlane = new Vector3f(object.getVerticesOfFace(object.getFace(face)).get(0));
        object.translateVertex(pointOnPlane);
        Vector3f axis = new Vector3f(object.getFace(face).getNormal());
        axis.rotate(object.getOrientation());
        Plane plane = new Plane(axis, pointOnPlane);

        float distance = Float.MAX_VALUE;
        for(int j = 0; j < clipPoints.size(); j++)
        {
            Vector3f point = new Vector3f(clipPoints.get(j));
            float current = plane.distanceToPoint(point);
            if(current < distance)
            {
                distance = current;
                safe.clear();
                safe.add(point);
            }
            else if(current == distance)
            {
                safe.add(point);
            }
        }
        return safe;
    }
    
    private boolean isSeparated2D(Vector2f projectionA, Vector2f projectionB, Vector3f axis, Vector3f offset, Vector3f velDelta, int index, Manifold manifold)
    {
        if(axis.lengthSquared() <= 0.0001f)
        {
            return true;
        }
                
        projectionA.x += Math.abs(offset.dot(axis));
        projectionA.y += Math.abs(offset.dot(axis));
        
        float d0 = projectionA.x - projectionB.y;
        float d1 = projectionB.x - projectionA.y;
        if(d0 > 0 || d1 > 0)
        {
            float velocity = velDelta.dot(axis);
            if(Math.abs(velocity) < 0.00001f)
            {
                return true;
            }
            
            float enterTime = (-d0 / velocity) * -1;
            float exitTime = (d1 / velocity) * -1;
            float temp;
            if(enterTime > exitTime)
            {
                temp = enterTime;
                enterTime = exitTime;
                exitTime = temp;
            }
            float timeOfImpact = (enterTime > 0) ? enterTime : exitTime;
            if(timeOfImpact < 0 || timeOfImpact > 1)
            {
                return true;
            }
            
            if(enterTime > manifold.getEnterTime())
            {
                manifold.setEnterTime(enterTime);
                manifold.setEnterNormal(axis);
                manifold.setType(index);
            }
            if(exitTime < manifold.getExitTime())
            {
                manifold.setExitTime(exitTime);
            }
            
            return false;
        }
        else
        {
            float overlap = (d0 > d1) ? d0 : d1;
            if(overlap > manifold.getPenetration())
            {
                manifold.setPenetration(overlap);
                manifold.setEnterNormal(axis);
                manifold.setType(index);
            }
            return false;
        }
    }
}
