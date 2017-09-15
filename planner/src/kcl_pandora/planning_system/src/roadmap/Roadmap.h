#ifndef KCL_Roadmap
#define KCL_Roadmap

namespace PandoraKCL
{

	struct Point3DQuat
	{
		double N; double E; double D;
		double x; double y; double z; double w;
	};

	/**
	 * Point3D; stores a full pose including orientation
	 */
	struct Point3D
	{
		double N; double E; double D;
		double p; double ya;

		bool isWaypoint;
		bool isConnected;

		Point3D() : N(0), E(0), D(0), p(0), ya(0) {};
		Point3D(float a,float b,float c) : N(a), E(b), D(c), p(0), ya(0) {};
		Point3D(float a,float b,float c,float d,float e) : N(a), E(b), D(c), p(d), ya(e) {};

		/* returns the magnitude of the point, ignoring depth */
		double mag2D() {
			return sqrt(N*N + E*E);
		}

		/* returns the magnitude of the point */
		double mag(){
			return sqrt(N*N + E*E + D*D);
		}

		/* rotates a position around the origin, (including yaw) */
		void rotate(double theta) {
			double oldN = N;
			N = N*cos(theta) - E*sin(theta);
			E = oldN*sin(theta) + E*cos(theta);
			ya += theta;
		}

		bool operator==(const Point3D & pp) {
			return N == pp.N && E == pp.E && D == pp.D && p == pp.p && ya == pp.ya;
		}
	};

	/**
	 *  An edge in the roadmap; used to represent a collision-free path between waypoints
	 */
	struct Connection
	{
		std::string start;
		std::string end;
		float distance;
		bool isVisible;
	};

	/* print a point3D like: "N E D p ya" */
	inline std::ostream & operator << (std::ostream & o,const Point3D & p) {
		o << p.N << " " << p.E << " " << p.D << " " << p.p << " " << p.ya;
		return o;
	}

	/*---------------------*/
	/* waypoint processing */
	/*---------------------*/

	/**
	 * Return the yaw value that from wp, points toward ip
	 */
	double findYaw(Point3D wp,Point3D ip) {
		double x = ip.N - wp.N;
		double y = ip.E - wp.E;
		double yaw = atan2(y,x);
		return yaw;
	}

	/**
	 * Compute the distance between two waypoints
	 */
	double computeDistance(Point3D p1, Point3D p2) {
		return (sqrt( (p1.N-p2.N)*(p1.N-p2.N) + (p1.E-p2.E)*(p1.E-p2.E) + (p1.D-p2.D)*(p1.D-p2.D) ));
	}

	/**
	 * Return an angle normalised between +/- PI
	 */
	double normalizeAngle( const double angle ) {
		return (angle + ( 2.0*3.141 * floor( ( 3.141-angle ) / ( 2.0*3.141 ) ) ) );
	}

	/**
	 * Return an angle normalised between +/- (1/2)*PI; for symmetric things like valves
	 */
	double normalizeHalfAngle( const double angle ) {
		return (angle + ( 3.141 * floor( ( 1.571-angle ) / ( 3.141 ) ) ) );
	}

	/**
	 * Compute the visibility/reachability of an inspection point from a waypoint
	 */
	double computeVisibility(Point3D wp, Point3D ip, double minD, double maxD, double angle) {

		if(computeDistance(wp,ip) < minD || computeDistance(wp,ip) > maxD)
			return 0;

		double vis = 0;
		double angle_from_origin = acos(std::min(std::max(
				(-0*(wp.E-ip.E) - 1*(wp.N-ip.N)) / computeDistance(ip,wp), -1.0),1.0));

		if(wp.E>ip.E) angle_from_origin = -angle_from_origin;
		double angle_from_normal = normalizeAngle(angle_from_origin - ip.ya);

		// within angle and distance
		if(abs(angle_from_normal) < angle)
			vis = 1;

		return vis;
	}

} // close namespace

#endif
