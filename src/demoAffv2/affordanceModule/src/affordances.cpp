#include <stdio.h>
#include <stdlib.h>

#include <iCub/affordances.h> // class's header file

PNL_USING

// class constructor
Affordances::Affordances()
{
}

// class destructor
Affordances::~Affordances()
{
CGraph *pGraph;
    delete pNaiveInf;
    delete pGraph;
    delete pBNet;
}


int Affordances::InitializeNetStructure(int numNodes, int *sizes, int *edge_mat)
{

    PNL_CHECK_LEFT_BORDER( numNodes, 1 );
    int i,j;

    cout << "Init net structure:\n";

    int index[] ={0,0};

    pGraph = CGraph::Create(0, NULL, NULL, NULL);
    PNL_CHECK_IF_MEMORY_ALLOCATED( pGraph );
    pGraph->AddNodes(numNodes);

    for (i = 0; i < numNodes; i++ ) {
	for (j = 0; j < numNodes; j++ ) {
	    if (edge_mat[i*numNodes+j]!=0){	
    		    pGraph->AddEdge(j, i, 1);
		}
	}
    }

    PNL_CHECK_IF_MEMORY_ALLOCATED( pGraph );

    if( !pGraph->IsDAG() )
    {
	    PNL_THROW( CInconsistentType, " the graph should be a DAG " );
    }

    if( !pGraph->IsTopologicallySorted() )
    {
	    PNL_THROW( CInconsistentType, 
		       " the graph should be sorted topologically " );
    }

    if (pGraph->NumberOfConnectivityComponents() > 1)
    {
	    PNL_THROW( CInconsistentType, " the graph should be linked " );
    }

    
    CNodeType *nodeTypes = new CNodeType [numNodes];
    int *nodeAssociation = new int[numNodes];
    
    for ( i = 0; i < numNodes; i++ )
    {
        nodeTypes[i].SetType(1, sizes[i]);
	nodeAssociation[i] = i;
    }

    pBNet = CBNet::Create( numNodes, numNodes, nodeTypes, nodeAssociation, pGraph );
    pMD = pBNet->GetModelDomain();
    
    myParams = new CFactor*[numNodes];
    nodeNumbers = new int[numNodes];
    domains = new int*[numNodes];

    intVector parents(0);
    for ( i = 0; i < numNodes; i++)
    {
        nodeNumbers[i] = pGraph->GetNumberOfParents(i) + 1;
        domains[i] = new int[nodeNumbers[i]];
        pGraph->GetParents(i, &parents);
        
        for ( j = 0; j < parents.size(); j++ )
            domains[i][j] = parents[j];
        domains[i][nodeNumbers[i]-1] = i;
    }
    
    pBNet->AllocFactors();
    
    for( i = 0; i < numNodes; i++ )
    {
        myParams[i] = CTabularCPD::Create( domains[i], 
            nodeNumbers[i], pMD);
    }

    delete [] nodeTypes;
    delete [] nodeAssociation;

}


int Affordances::InitializeParameters(int node, float *CPD)
{	
	int j,k;
	float *data;
	int size_data;
	int num_states_node;
	int num_blocks;
	intVector size_nodes(0);
	float belief, sum_beliefs;

	cout<<"Compute size\n";
	// compute size of the table
	size_data = 1;
	size_nodes.resize(0);
	
	for ( j = 0; j < nodeNumbers[node]; j++ )
	{
		cout <<"goood\n";
		cout << "node" << node << "j:" << j<< "size" << pBNet->GetNodeType( domains[node][j])->GetNodeSize() <<"\n";
	    size_nodes.push_back(pBNet->GetNodeType( domains[node][j])->GetNodeSize());
	    size_data *= size_nodes[j];
	}
	
	cout << "size data: " << size_data <<"\n";
	num_states_node = size_nodes[size_nodes.size() - 1];
	num_blocks = size_data / num_states_node;
	data = new float[size_data];

	cout<<"Copy table\n";
	
	for ( j = 0; j < num_blocks; j++ ) {
	    sum_beliefs = 0.0;
	    for ( k = 0; k < num_states_node; k++ )
	    {
		    cout << CPD[j * num_states_node + k]<<" ";
		    data[j * num_states_node + k] = CPD[j * num_states_node + k];
		    sum_beliefs += CPD[j * num_states_node + k];
	    }
	    if (sum_beliefs!=1) {
		cout << "Wrong CPD for node " << node << "sumbelief: "<< sum_beliefs<<"\n";
		return (-1);
	    }
	}
	

	cout<<"Attach factor\n";
	myParams[node]->AllocMatrix(data, matTable);
	pBNet->AttachFactor(myParams[node]);
	
	cout<<"End\n";

}

int Affordances::SolveQueryPerfectObs(int *evidence, int numQueryNds, int *queryNds, float *outP)
{
	
   int i, nObsNodes=0, nSize;
   int nNodes;
   nNodes=pGraph->GetNumberOfNodes();
   
   for (i=0; i<nNodes; i++)
	   cout << "ev["<<i<<"]="<<evidence[i]<<"\n";
	
   
   //get content of Graph
   pBNet->GetGraph()->Dump();
   
   //create simple evidence for node 0 from BNet
   CEvidence* pEvidForWS;
   
   
   int obsNds[nNodes];
   valueVector obsVals;
   
   
   for (i=0; i<nNodes; i++){
      nSize=pBNet->GetNodeType(i)->GetNodeSize();

      cout <<"Node :"<< i << " Size :" << nSize<<"\n";
      cout << "Evidence: " << evidence[i] <<"\n";
      if (evidence[i]!=-1) {
         if ( !(evidence[i]>=0 && evidence[i]<nSize)){
             std::cout<< "error: incorrect value for node " << i;
         }
         else {
		 cout << "observed value for node" << i <<":" << evidence[i] << "\n";
              obsNds[nObsNodes]=i;
              nObsNodes++;
              obsVals.resize(nObsNodes);
              obsVals[nObsNodes-1].SetInt(evidence[i]);
         }

      }
   }

   cout << "create evidence\n";
   pEvidForWS = CEvidence::Create( pBNet, nObsNodes, obsNds, obsVals );
   cout << "Enter evidence\n";
   pNaiveInf->EnterEvidence( pEvidForWS );

   cout << "Get marginals\n";
   //get a marginal for query set of nodes
   pNaiveInf->MarginalNodes( queryNds, numQueryNds );
   
   cout << "Marginals done\n";
   
   const CPotential* pMarg = pNaiveInf->GetQueryJPD();
   
   cout << "after query\n";
   
   intVector obsNodes;
   pConstValueVector obsVls;
   pEvidForWS->GetObsNodesWithValues(&obsNodes, &obsVls);
   
   for( i = 0; i < obsNodes.size(); i++ )
   {
       std::cout<<" observed value for node "<<obsNds[i];
       std::cout<<" is "<<obsVls[i]->GetInt()<<std::endl;
   }
   
   int nnodes;
   const int* domain;
   pMarg->GetDomain( &nnodes, &domain );
   std::cout<<" inference results: \n";
   std::cout<<" probability distribution for nodes [ ";
   for( i = 0; i < nnodes; i++ )
   {
       std::cout <<domain[i] <<" ";
   }
   
   std::cout<<"]"<<std::endl;
   CMatrix<float>* pMat = pMarg->GetMatrix(matTable);
   // graphical model hase been created using dense matrix
   // so, the marginal is also dense
   EMatrixClass type = pMat->GetMatrixClass();
   if( ! ( type == mcDense || type == mcNumericDense || type == mc2DNumericDense ) )
   {
       assert(0);
   }
   
   int nEl;
   const float* data;
   static_cast<CNumericDenseMatrix<float>*>(pMat)->GetRawData(&nEl, &data);
   cout << "nEl: "<< nEl<<"\n";
   for( i = 0; i < nEl; i++ )
   {
       std::cout<<" "<<data[i];
       outP[i]=data[i]; 
   }
   
   std::cout<<std::endl;
   delete pEvidForWS;
}

int Affordances::CreateEngine() {
   cout << "create engine\n";
   
   //create Naive inference for BNet
   pNaiveInf = CNaiveInfEngine::Create( pBNet );
}


