using UnityEngine;
using System.Collections;
using UnityEditor.Rendering.Analytics;
// using System.Numerics;

// * Math
public static class MatrixExtensions
{
	// Add two matrices
	public static Matrix4x4 Add(this Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 result = new Matrix4x4();
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result[i, j] = a[i, j] + b[i, j];
			}
		}
		return result;
	}

	// Add two quaternions
	public static Quaternion Add(this Quaternion a, Quaternion b)
	{
		Quaternion result = new Quaternion();
		result.x = a.x + b.x;
		result.y = a.y + b.y;
		result.z = a.z + b.z;
		result.w = a.w + b.w;
		return result;
	}

	// cross Vector3 and Vector3
	public static Vector3 Cross(this Vector3 a, Vector3 b)
	{
		Vector3 result = new Vector3();
		result.x = a.y * b.z - a.z * b.y;
		result.y = a.z * b.x - a.x * b.z;
		result.z = a.x * b.y - a.y * b.x;
		return result;
	}

	// dot Vector3 and Vector3^T
	public static Matrix4x4 dot_T(this Vector3 a, Vector3 b)
	{
		Matrix4x4 result = new Matrix4x4();
		result[0, 0] = a.x * b.x;
		result[0, 1] = a.x * b.y;
		result[0, 2] = a.x * b.z;
		result[1, 0] = a.y * b.x;
		result[1, 1] = a.y * b.y;
		result[1, 2] = a.y * b.z;
		result[2, 0] = a.z * b.x;
		result[2, 1] = a.z * b.y;
		result[2, 2] = a.z * b.z;
		return result;
	}
}


public class Rigid_Bunny : MonoBehaviour
{
	// global settings
	bool launched = false;
	float dt = 0.01f;

	// initialize the object status
	Vector3 v = new Vector3(0, 0, 0);   // velocity
	Vector3 w = new Vector3(0, 0, 0);   // angular velocity
	float mass;                  // mass
	float mass_inv;              // inverse mass
	Matrix4x4 I_ref;                            // reference inertia

	// initialize the ground status
	public GameObject ground;
	public GameObject wall;

	// physical parameters
	float gravity = 9.8f;                    // gravity
	float linear_decay = 0.999f;                // for velocity decay
	float angular_decay = 0.98f;
	float restitution = 0.5f;                   // for collision
	float normal_decay = 0.5f;
	float targent_decay = 0.5f;
	float damping = 0.8f;                     // for angular velocity decay


	// Use this for initialization
	void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m = 1;
		mass = 0;
		for (int i = 0; i < vertices.Length; i++)
		{
			mass += m;
			float diag = m * vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
		}
		I_ref[3, 3] = 1;

		Debug.Log("I_ref: " + I_ref);

		mass_inv = 1 / mass;
	}

	// * Math
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = 0;
		A[0, 1] = -a[2];
		A[0, 2] = a[1];
		A[1, 0] = a[2];
		A[1, 1] = 0;
		A[1, 2] = -a[0];
		A[2, 0] = -a[1];
		A[2, 1] = a[0];
		A[2, 2] = 0;
		A[3, 3] = 1;
		return A;
	}
	float SDF(Vector3 x, ref Vector3 P, ref Vector3 N)
	{
		//Get the signed distance function value of the plane

		Vector3 P2x = x - P;
		float d = Vector3.Dot(P2x, N);

		return d;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		// ? debug: print the P and N
		// Debug.Log("P: " + P);
		// Debug.Log("N: " + N);

		// get the position and orientation of the object
		Vector3 x = transform.position;
		Quaternion q = transform.rotation;

		// collision average velocity and position
		int cnt = 0;
		Vector3 r_avg = Vector3.zero;
		Vector3 v_avg = Vector3.zero;

		// get the global I
		Matrix4x4 qMatrix = Matrix4x4.Rotate(q);
		Matrix4x4 I = qMatrix * I_ref * qMatrix.transpose;
		Matrix4x4 I_inv = I.inverse;

		// get the all vertices of the object
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		// for each vertex, check if it is inside the plane
		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 x_i = x + q * vertices[i];
			Vector3 tmp_r_i = q * vertices[i];

			// get the signed distance function value
			float d = SDF(x_i, ref P, ref N);

			// if the vertex is inside the plane
			if (d < 0.05f)
			{
				// get the velocity of the vertex
				Vector3 v_i = this.v + Vector3.Cross(this.w, tmp_r_i);

				// if the velocity is not going to the plane
				if (Vector3.Dot(v_i, N) > 0)
				{
					continue;
				}

				// get the collision average velocity and position
				r_avg += tmp_r_i;
				v_avg += v_i;
				cnt++;

			}
		}

		// * implement the collision impulse
		// do average
		if (cnt == 0)
		{
			return;
		}

		r_avg /= cnt;
		v_avg /= cnt;

		// get the vn and vt
		Vector3 v_n = Vector3.Dot(v_avg, N) * N;
		Vector3 v_t = v_avg - v_n;

		// get the cross matrix
		Vector3 r_i = r_avg;
		Matrix4x4 Rri = Get_Cross_Matrix(r_i);

		// get the new velocity of the vertex
		float a = Mathf.Max(0.0f, 1 - targent_decay * (1 + normal_decay) * v_n.magnitude / v_t.magnitude);
		Vector3 v_avg_new = -normal_decay * v_n + a * v_t;

		// get the impulse
		Matrix4x4 K = MatrixExtensions.Add(Matrix4x4.Scale(new Vector3(mass_inv, mass_inv, mass_inv)), Rri * I_inv * Rri.transpose);
		Vector3 impulse = K.inverse * (v_avg_new - v_avg);

		// update the velocity
		this.v += impulse * mass_inv;
		this.w += (Vector3)(I_inv * Rri * impulse);
	}

	// Update is called once per frame
	void Update()
	{
		//Game Control
		if (Input.GetKey("r"))
		{
			transform.position = new Vector3(0, 2.0f, 0);
			v = new Vector3(0, 0, 5);
			restitution = 0.5f;
			launched = false;
		}
		if (Input.GetKey("l"))
		{
			v = new Vector3(0, 0, 5);
			w = new Vector3(0, 0, 0);
			launched = true;
		}

		// Part I: Update velocities
		v.y -= gravity * dt;

		// Part II: Collision Impulse
		Collision_Impulse(ground.transform.position, ground.transform.up);
		Collision_Impulse(wall.transform.position, wall.transform.up);

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x = transform.position + damping * v * dt;
		//Update angular status
		float theta = damping * 0.5f * dt;
		Quaternion deltaRotation = new Quaternion(w.x * theta, w.y * theta, w.z * theta, 0);
		Quaternion q = MatrixExtensions.Add(transform.rotation, deltaRotation * transform.rotation).normalized;

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
