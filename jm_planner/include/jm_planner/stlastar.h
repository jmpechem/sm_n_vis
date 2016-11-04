#include <iostream>
#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <set>
#include <vector>
#include <cfloat>

using namespace std;
template <class T> class AStarState;
template <class UserState> class AStarSearch
{
public:
	enum
	{
		SEARCH_STATE_NOT_INITIALISED,
		SEARCH_STATE_SEARCHING,
		SEARCH_STATE_SUCCEEDED,
		SEARCH_STATE_FAILED,
		SEARCH_STATE_OUT_OF_MEMORY,
		SEARCH_STATE_INVALID
	};
	public:
	class Node
	{
		public:
			Node *parent;
			Node *child;			
			float g; 
			float h; 
			float f; 

			Node() :
				parent( 0 ),
				child( 0 ),
				g( 0.0f ),
				h( 0.0f ),
				f( 0.0f )
			{			
			}

			UserState m_UserState;
	};
	class HeapCompare_f 
	{
		public:

			bool operator() ( const Node *x, const Node *y ) const
			{
				return x->f > y->f;
			}
	};
	public:
	AStarSearch() :
		m_State( SEARCH_STATE_NOT_INITIALISED ),
		m_CurrentSolutionNode( NULL ),
		m_AllocateNodeCount(0),
		m_CancelRequest( false )
	{
	}
	AStarSearch( int MaxNodes ) :
		m_State( SEARCH_STATE_NOT_INITIALISED ),
		m_CurrentSolutionNode( NULL ),
		m_AllocateNodeCount(0),
		m_CancelRequest( false )
	{
	}	
	void CancelSearch()
	{
		m_CancelRequest = true;
	}
	void SetStartAndGoalStates( UserState &Start, UserState &Goal )
	{
		m_CancelRequest = false;
		m_Start = AllocateNode();
		m_Goal = AllocateNode();
		assert((m_Start != NULL && m_Goal != NULL));		
		m_Start->m_UserState = Start;
		m_Goal->m_UserState = Goal;
		m_State = SEARCH_STATE_SEARCHING;
		m_Start->g = 0; 
		m_Start->h = m_Start->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
		m_Start->f = m_Start->g + m_Start->h;
		m_Start->parent = 0;
		m_OpenList.push_back( m_Start );		
		push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
		m_Steps = 0;
	}

	unsigned int SearchStep()
	{
		assert( (m_State > SEARCH_STATE_NOT_INITIALISED) && (m_State < SEARCH_STATE_INVALID) );
		if((m_State == SEARCH_STATE_SUCCEEDED) || (m_State == SEARCH_STATE_FAILED))
		{
			return m_State; 
		}
		if( m_OpenList.empty() || m_CancelRequest )
		{
			FreeAllNodes();
			m_State = SEARCH_STATE_FAILED;
			return m_State;
		}
		m_Steps ++;
		Node *n = m_OpenList.front();
		pop_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
		m_OpenList.pop_back();
		if( n->m_UserState.IsGoal( m_Goal->m_UserState ) )
		{
			m_Goal->parent = n->parent;
			m_Goal->g = n->g;
			if( false == n->m_UserState.IsSameState( m_Start->m_UserState ) )
			{
				FreeNode( n );
				Node *nodeChild = m_Goal;
				Node *nodeParent = m_Goal->parent;
				do 
				{
					nodeParent->child = nodeChild;
					nodeChild = nodeParent;
					nodeParent = nodeParent->parent;				
				} 
				while( nodeChild != m_Start );
			}
			FreeUnusedNodes();
			m_State = SEARCH_STATE_SUCCEEDED;
			return m_State;
		}
		else
		{
			m_Successors.clear();
			bool ret = n->m_UserState.GetSuccessors( this, n->parent ? &n->parent->m_UserState : NULL ); 
			if( !ret )
			{
			    typename vector< Node * >::iterator successor;
				for( successor = m_Successors.begin(); successor != m_Successors.end(); successor ++ )
				{
					FreeNode( (*successor) );
				}
				m_Successors.clear();
				FreeAllNodes();
				m_State = SEARCH_STATE_OUT_OF_MEMORY;
				return m_State;
			}
			for( typename vector< Node * >::iterator successor = m_Successors.begin(); successor != m_Successors.end(); successor ++ )
			{

				float newg = n->g + n->m_UserState.GetCost( (*successor)->m_UserState );
				typename vector< Node * >::iterator openlist_result;
				for( openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result ++ )
				{
					if( (*openlist_result)->m_UserState.IsSameState( (*successor)->m_UserState ) )
					{
						break;					
					}
				}

				if( openlist_result != m_OpenList.end() )
				{
					if( (*openlist_result)->g <= newg )
					{
						FreeNode( (*successor) );
						continue;
					}
				}

				typename vector< Node * >::iterator closedlist_result;

				for( closedlist_result = m_ClosedList.begin(); closedlist_result != m_ClosedList.end(); closedlist_result ++ )
				{
					if( (*closedlist_result)->m_UserState.IsSameState( (*successor)->m_UserState ) )
					{
						break;					
					}
				}

				if( closedlist_result != m_ClosedList.end() )
				{

					if( (*closedlist_result)->g <= newg )
					{
						FreeNode( (*successor) );
						continue;
					}
				}
				(*successor)->parent = n;
				(*successor)->g = newg;
				(*successor)->h = (*successor)->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
				(*successor)->f = (*successor)->g + (*successor)->h;
				if( closedlist_result != m_ClosedList.end() )
				{
					FreeNode(  (*closedlist_result) ); 
					m_ClosedList.erase( closedlist_result );
				}
				if( openlist_result != m_OpenList.end() )
				{	   
					FreeNode( (*openlist_result) ); 
			   		m_OpenList.erase( openlist_result );
					make_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );			
				}
				m_OpenList.push_back( (*successor) );
				push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
			}
			m_ClosedList.push_back( n );
		}
 		return m_State;
	}
	bool AddSuccessor( UserState &State )
	{
		Node *node = AllocateNode();

		if( node )
		{
			node->m_UserState = State;

			m_Successors.push_back( node );

			return true;
		}

		return false;
	}
	void FreeSolutionNodes()
	{
		Node *n = m_Start;

		if( m_Start->child )
		{
			do
			{
				Node *del = n;
				n = n->child;
				FreeNode( del );

				del = NULL;

			} while( n != m_Goal );

			FreeNode( n );

		}
		else
		{
			FreeNode( m_Start );
			FreeNode( m_Goal );
		}

	}
	UserState *GetSolutionStart()
	{
		m_CurrentSolutionNode = m_Start;
		if( m_Start )
		{
			return &m_Start->m_UserState;
		}
		else
		{
			return NULL;
		}
	}
	UserState *GetSolutionNext()
	{
		if( m_CurrentSolutionNode )
		{
			if( m_CurrentSolutionNode->child )
			{

				Node *child = m_CurrentSolutionNode->child;

				m_CurrentSolutionNode = m_CurrentSolutionNode->child;

				return &child->m_UserState;
			}
		}

		return NULL;
	}
	UserState *GetSolutionEnd()
	{
		m_CurrentSolutionNode = m_Goal;
		if( m_Goal )
		{
			return &m_Goal->m_UserState;
		}
		else
		{
			return NULL;
		}
	}
	UserState *GetSolutionPrev()
	{
		if( m_CurrentSolutionNode )
		{
			if( m_CurrentSolutionNode->parent )
			{

				Node *parent = m_CurrentSolutionNode->parent;

				m_CurrentSolutionNode = m_CurrentSolutionNode->parent;

				return &parent->m_UserState;
			}
		}

		return NULL;
	}
	float GetSolutionCost()
	{
		if( m_Goal && m_State == SEARCH_STATE_SUCCEEDED )
		{
			return m_Goal->g;
		}
		else
		{
			return FLT_MAX;
		}
	}

	UserState *GetOpenListStart()
	{
		float f,g,h;
		return GetOpenListStart( f,g,h );
	}

	UserState *GetOpenListStart( float &f, float &g, float &h )
	{
		iterDbgOpen = m_OpenList.begin();
		if( iterDbgOpen != m_OpenList.end() )
		{
			f = (*iterDbgOpen)->f;
			g = (*iterDbgOpen)->g;
			h = (*iterDbgOpen)->h;
			return &(*iterDbgOpen)->m_UserState;
		}
		return NULL;
	}

	UserState *GetOpenListNext()
	{
		float f,g,h;
		return GetOpenListNext( f,g,h );
	}

	UserState *GetOpenListNext( float &f, float &g, float &h )
	{
		iterDbgOpen++;
		if( iterDbgOpen != m_OpenList.end() )
		{
			f = (*iterDbgOpen)->f;
			g = (*iterDbgOpen)->g;
			h = (*iterDbgOpen)->h;
			return &(*iterDbgOpen)->m_UserState;
		}

		return NULL;
	}

	UserState *GetClosedListStart()
	{
		float f,g,h;
		return GetClosedListStart( f,g,h );
	}

	UserState *GetClosedListStart( float &f, float &g, float &h )
	{
		iterDbgClosed = m_ClosedList.begin();
		if( iterDbgClosed != m_ClosedList.end() )
		{
			f = (*iterDbgClosed)->f;
			g = (*iterDbgClosed)->g;
			h = (*iterDbgClosed)->h;

			return &(*iterDbgClosed)->m_UserState;
		}

		return NULL;
	}
	UserState *GetClosedListNext()
	{
		float f,g,h;
		return GetClosedListNext( f,g,h );
	}
	UserState *GetClosedListNext( float &f, float &g, float &h )
	{
		iterDbgClosed++;
		if( iterDbgClosed != m_ClosedList.end() )
		{
			f = (*iterDbgClosed)->f;
			g = (*iterDbgClosed)->g;
			h = (*iterDbgClosed)->h;

			return &(*iterDbgClosed)->m_UserState;
		}

		return NULL;
	}
	int GetStepCount() { return m_Steps; }
	void EnsureMemoryFreed()
	{
	}
	private:
	void FreeAllNodes()
	{
		typename vector< Node * >::iterator iterOpen = m_OpenList.begin();
		while( iterOpen != m_OpenList.end() )
		{
			Node *n = (*iterOpen);
			FreeNode( n );

			iterOpen ++;
		}
		m_OpenList.clear();
		typename vector< Node * >::iterator iterClosed;
		for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ )
		{
			Node *n = (*iterClosed);
			FreeNode( n );
		}
		m_ClosedList.clear();
		FreeNode(m_Goal);
	}
	void FreeUnusedNodes()
	{
		typename vector< Node * >::iterator iterOpen = m_OpenList.begin();

		while( iterOpen != m_OpenList.end() )
		{
			Node *n = (*iterOpen);

			if( !n->child )
			{
				FreeNode( n );

				n = NULL;
			}

			iterOpen ++;
		}
		m_OpenList.clear();
		typename vector< Node * >::iterator iterClosed;
		for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ )
		{
			Node *n = (*iterClosed);
			if( !n->child )
			{
				FreeNode( n );
				n = NULL;
			}
		}
		m_ClosedList.clear();
	}
	Node *AllocateNode()
	{
		Node *p = new Node;
		return p;
	}
	void FreeNode( Node *node )
	{
		m_AllocateNodeCount --;
		delete node;
	}
private:
	vector< Node *> m_OpenList;
	vector< Node * > m_ClosedList; 
	vector< Node * > m_Successors;
	unsigned int m_State;
	int m_Steps;	
	Node *m_Start;
	Node *m_Goal;
	Node *m_CurrentSolutionNode;
	typename vector< Node * >::iterator iterDbgOpen;
	typename vector< Node * >::iterator iterDbgClosed;
	int m_AllocateNodeCount;	
	bool m_CancelRequest;

};

template <class T> class AStarState
{
public:
	virtual ~AStarState() {}
	virtual float GoalDistanceEstimate( T &nodeGoal ) = 0; 
	virtual bool IsGoal( T &nodeGoal ) = 0; 
	virtual bool GetSuccessors( AStarSearch<T> *astarsearch, T *parent_node ) = 0;
	virtual float GetCost( T &successor ) = 0; 
	virtual bool IsSameState( T &rhs ) = 0; 
};
