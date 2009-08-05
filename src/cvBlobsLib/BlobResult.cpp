/************************************************************************
  			BlobResult.cpp
  			
FUNCIONALITAT: Implementaci�de la classe CBlobResult
AUTOR: Inspecta S.L.
MODIFICACIONS (Modificaci� Autor, Data):
LICENSE: See README.TXT
 
**************************************************************************/

#include <limits.h>
#include <stdio.h>
#include <functional>
#include <algorithm>
#include <BlobResult.h>
#include <BlobExtraction.h>
#undef _DEBUG
#ifdef _DEBUG
	#include <afx.h>			//suport per a CStrings
	#include <afxwin.h>			//suport per a AfxMessageBox
#endif

/**************************************************************************
		Constructors / Destructors
**************************************************************************/


/**
- FUNCI� CBlobResult
- FUNCIONALITAT: Constructor estandard.
- PAR�ETRES:
- RESULTAT:
- Crea un CBlobResult sense cap blob
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 20-07-2004.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: CBlobResult
- FUNCTIONALITY: Standard constructor
- PARAMETERS:
- RESULT:
	- creates an empty set of blobs
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlobResult::CBlobResult()
{
	m_blobs = blob_vector();
}

/**
- FUNCI� CBlobResult
- FUNCIONALITAT: Constructor a partir d'una imatge. Inicialitza la seq�cia de blobs 
			   amb els blobs resultants de l'an�isi de blobs de la imatge.
- PAR�ETRES:
	- source: imatge d'on s'extreuran els blobs
	- mask: m�cara a aplicar. Nom� es calcularan els blobs on la m�cara sigui 
			diferent de 0. Els blobs que toquin a un pixel 0 de la m�cara seran 
			considerats exteriors.
	- threshold: llindar que s'aplicar�a la imatge source abans de calcular els blobs
	- findmoments: indica si s'han de calcular els moments de cada blob
- RESULTAT:
	- objecte CBlobResult amb els blobs de la imatge source
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: CBlob
- FUNCTIONALITY: Constructor from an image. Fills an object with all the blobs in
	the image
- PARAMETERS:
	- source: image to extract the blobs from
	- mask: optional mask to apply. The blobs will be extracted where the mask is
			not 0. All the neighbouring blobs where the mask is 0 will be extern blobs
	- threshold: threshold level to apply to the image before computing blobs
	- findmoments: true to calculate the blob moments (slower)
- RESULT:
	- object with all the blobs in the image. It throws an EXCEPCIO_CALCUL_BLOBS
	  if some error appears in the BlobAnalysis function
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlobResult::CBlobResult(IplImage *source, IplImage *mask, int threshold, bool findmoments)
{
	bool success;

	try
	{
		// cridem la funci�amb el marc a true=1=blanc (aix�no unir�els blobs externs)
		success = BlobAnalysis(source,(uchar)threshold,mask,true,findmoments, m_blobs );
	}
	catch(...)
	{
		success = false;
	}

	if( !success ) throw EXCEPCIO_CALCUL_BLOBS;
}

/**
- FUNCI� CBlobResult
- FUNCIONALITAT: Constructor de c�ia. Inicialitza la seq�cia de blobs 
			   amb els blobs del par�etre.
- PAR�ETRES:
	- source: objecte que es copiar�
- RESULTAT:
	- objecte CBlobResult amb els blobs de l'objecte source
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: CBlobResult
- FUNCTIONALITY: Copy constructor
- PARAMETERS:
	- source: object to copy
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlobResult::CBlobResult( const CBlobResult &source )
{
	m_blobs = blob_vector( source.GetNumBlobs() );
	
	// creem el nou a partir del passat com a par�etre
	m_blobs = blob_vector( source.GetNumBlobs() );
	// copiem els blobs de l'origen a l'actual
	blob_vector::const_iterator pBlobsSrc = source.m_blobs.begin();
	blob_vector::iterator pBlobsDst = m_blobs.begin();

	while( pBlobsSrc != source.m_blobs.end() )
	{
		// no podem cridar a l'operador = ja que blob_vector � un 
		// vector de CBlob*. Per tant, creem un blob nou a partir del
		// blob original
		*pBlobsDst = new CBlob(**pBlobsSrc);
		pBlobsSrc++;
		pBlobsDst++;
	}
}



/**
- FUNCI� ~CBlobResult
- FUNCIONALITAT: Destructor estandard.
- PAR�ETRES:
- RESULTAT:
	- Allibera la mem�ia reservada de cadascun dels blobs de la classe
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: ~CBlobResult
- FUNCTIONALITY: Destructor
- PARAMETERS:
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlobResult::~CBlobResult()
{
	ClearBlobs();
}

/**************************************************************************
		Operadors / Operators
**************************************************************************/


/**
- FUNCI� operador =
- FUNCIONALITAT: Assigna un objecte source a l'actual
- PAR�ETRES:
	- source: objecte a assignar
- RESULTAT:
	- Substitueix els blobs actuals per els de l'objecte source
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: Assigment operator
- FUNCTIONALITY: 
- PARAMETERS:
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlobResult& CBlobResult::operator=(const CBlobResult& source)
{
	// si ja s� el mateix, no cal fer res
	if (this != &source)
	{
		// alliberem el conjunt de blobs antic
		for( int i = 0; i < GetNumBlobs(); i++ )
		{
			delete m_blobs[i];
		}
		m_blobs.clear();
		// creem el nou a partir del passat com a par�etre
		m_blobs = blob_vector( source.GetNumBlobs() );
		// copiem els blobs de l'origen a l'actual
		blob_vector::const_iterator pBlobsSrc = source.m_blobs.begin();
		blob_vector::iterator pBlobsDst = m_blobs.begin();

		while( pBlobsSrc != source.m_blobs.end() )
		{
			// no podem cridar a l'operador = ja que blob_vector � un 
			// vector de CBlob*. Per tant, creem un blob nou a partir del
			// blob original
			*pBlobsDst = new CBlob(**pBlobsSrc);
			pBlobsSrc++;
			pBlobsDst++;
		}
	}
	return *this;
}


/**
- FUNCI� operador +
- FUNCIONALITAT: Concatena els blobs de dos CBlobResult
- PAR�ETRES:
	- source: d'on s'agafaran els blobs afegits a l'actual
- RESULTAT:
	- retorna un nou CBlobResult amb els dos CBlobResult concatenats
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- NOTA: per la implementaci� els blobs del par�etre es posen en ordre invers
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: + operator
- FUNCTIONALITY: Joins the blobs in source with the current ones
- PARAMETERS:
	- source: object to copy the blobs
- RESULT:
	- object with the actual blobs and the source blobs
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlobResult CBlobResult::operator+( const CBlobResult& source )
{	
	//creem el resultat a partir dels blobs actuals
	CBlobResult resultat( *this );
	
	// reservem mem�ia per als nous blobs
	resultat.m_blobs.resize( resultat.GetNumBlobs() + source.GetNumBlobs() );

	// declarem els iterador per rec�rer els blobs d'origen i desti
	blob_vector::const_iterator pBlobsSrc = source.m_blobs.begin();
	blob_vector::iterator pBlobsDst = resultat.m_blobs.end();

	// insertem els blobs de l'origen a l'actual
	while( pBlobsSrc != source.m_blobs.end() )
	{
		pBlobsDst--;
		*pBlobsDst = new CBlob(**pBlobsSrc);
		pBlobsSrc++;
	}
	
	return resultat;
}

/**************************************************************************
		Operacions / Operations
**************************************************************************/

/**
- FUNCI� AddBlob
- FUNCIONALITAT: Afegeix un blob al conjunt
- PAR�ETRES:
	- blob: blob a afegir
- RESULTAT:
	- modifica el conjunt de blobs actual
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 2006/03/01
- MODIFICACI� Data. Autor. Descripci�
*/
void CBlobResult::AddBlob( CBlob *blob )
{
	if( blob != NULL )
		m_blobs.push_back( new CBlob( blob ) );
}


#ifdef MATRIXCV_ACTIU

/**
- FUNCI� GetResult
- FUNCIONALITAT: Calcula el resultat especificat sobre tots els blobs de la classe
- PAR�ETRES:
	- evaluador: Qualsevol objecte derivat de COperadorBlob
- RESULTAT:
	- Retorna un array de double's amb el resultat per cada blob
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: GetResult
- FUNCTIONALITY: Computes the function evaluador on all the blobs of the class
				 and returns a vector with the result
- PARAMETERS:
	- evaluador: function to apply to each blob (any object derived from the 
				 COperadorBlob class )
- RESULT:
	- vector with all the results in the same order as the blobs
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
double_vector CBlobResult::GetResult( funcio_calculBlob *evaluador ) const
{
	if( GetNumBlobs() <= 0 )
	{
		return double_vector();
	}

	// definim el resultat
	double_vector result = double_vector( GetNumBlobs() );
	// i iteradors sobre els blobs i el resultat
	double_vector::iterator itResult = result.GetIterator();
	blob_vector::const_iterator itBlobs = m_blobs.begin();

	// avaluem la funci�en tots els blobs
	while( itBlobs != m_blobs.end() )
	{
		*itResult = (*evaluador)(**itBlobs);
		itBlobs++;
		itResult++;
	}
	return result;
}
#endif

/**
- FUNCI� GetSTLResult
- FUNCIONALITAT: Calcula el resultat especificat sobre tots els blobs de la classe
- PAR�ETRES:
	- evaluador: Qualsevol objecte derivat de COperadorBlob
- RESULTAT:
	- Retorna un array de double's STL amb el resultat per cada blob
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: GetResult
- FUNCTIONALITY: Computes the function evaluador on all the blobs of the class
				 and returns a vector with the result
- PARAMETERS:
	- evaluador: function to apply to each blob (any object derived from the 
				 COperadorBlob class )
- RESULT:
	- vector with all the results in the same order as the blobs
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
double_stl_vector CBlobResult::GetSTLResult( funcio_calculBlob *evaluador ) const
{
	if( GetNumBlobs() <= 0 )
	{
		return double_stl_vector();
	}

	// definim el resultat
	double_stl_vector result = double_stl_vector( GetNumBlobs() );
	// i iteradors sobre els blobs i el resultat
	double_stl_vector::iterator itResult = result.begin();
	blob_vector::const_iterator itBlobs = m_blobs.begin();

	// avaluem la funci�en tots els blobs
	while( itBlobs != m_blobs.end() )
	{
		*itResult = (*evaluador)(**itBlobs);
		itBlobs++;
		itResult++;
	}
	return result;
}

/**
- FUNCI� GetNumber
- FUNCIONALITAT: Calcula el resultat especificat sobre un nic blob de la classe
- PAR�ETRES:
	- evaluador: Qualsevol objecte derivat de COperadorBlob
	- indexblob: nmero de blob del que volem calcular el resultat.
- RESULTAT:
	- Retorna un double amb el resultat
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: GetNumber
- FUNCTIONALITY: Computes the function evaluador on a blob of the class
- PARAMETERS:
	- indexBlob: index of the blob to compute the function
	- evaluador: function to apply to each blob (any object derived from the 
				 COperadorBlob class )
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
double CBlobResult::GetNumber( int indexBlob, funcio_calculBlob *evaluador ) const
{
	if( indexBlob < 0 || indexBlob >= GetNumBlobs() )
		RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );
	return (*evaluador)( *m_blobs[indexBlob] );
}

/////////////////////////// FILTRAT DE BLOBS ////////////////////////////////////

/**
- FUNCI� Filter
- FUNCIONALITAT: Filtra els blobs de la classe i deixa el resultat amb nom� 
			   els blobs que han passat el filtre.
			   El filtrat es basa en especificar condicions sobre un resultat dels blobs
			   i seleccionar (o excloure) aquells blobs que no compleixen una determinada
			   condicio
- PAR�ETRES:
	- dst: variable per deixar els blobs filtrats
	- filterAction:	acci�de filtrat. Incloure els blobs trobats (B_INCLUDE),
				    o excloure els blobs trobats (B_EXCLUDE)
	- evaluador: Funci�per evaluar els blobs (qualsevol objecte derivat de COperadorBlob
	- Condition: tipus de condici�que ha de superar la mesura (FilterType) 
				 sobre cada blob per a ser considerat.
				    B_EQUAL,B_NOT_EQUAL,B_GREATER,B_LESS,B_GREATER_OR_EQUAL,
				    B_LESS_OR_EQUAL,B_INSIDE,B_OUTSIDE
	- LowLimit:  valor num�ic per a la comparaci�(Condition) de la mesura (FilterType)
	- HighLimit: valor num�ic per a la comparaci�(Condition) de la mesura (FilterType)
				 (nom� t�sentit per a aquelles condicions que tenen dos valors 
				 (B_INSIDE, per exemple).
- RESULTAT:
	- Deixa els blobs resultants del filtrat a destination
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/**
- FUNCTION: Filter
- FUNCTIONALITY: Get some blobs from the class based on conditions on measures
				 of the blobs. 
- PARAMETERS:
	- dst: where to store the selected blobs
	- filterAction:	B_INCLUDE: include the blobs which pass the filter in the result 
				    B_EXCLUDE: exclude the blobs which pass the filter in the result 
	- evaluador: Object to evaluate the blob
	- Condition: How to decide if  the result returned by evaluador on each blob
				 is included or not. It can be:
				    B_EQUAL,B_NOT_EQUAL,B_GREATER,B_LESS,B_GREATER_OR_EQUAL,
				    B_LESS_OR_EQUAL,B_INSIDE,B_OUTSIDE
	- LowLimit:  numerical value to evaluate the Condition on evaluador(blob)
	- HighLimit: numerical value to evaluate the Condition on evaluador(blob).
				 Only useful for B_INSIDE and B_OUTSIDE
- RESULT:
	- It returns on dst the blobs that accomplish (B_INCLUDE) or discards (B_EXCLUDE)
	  the Condition on the result returned by evaluador on each blob
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
void CBlobResult::Filter(CBlobResult &dst, 
						 int filterAction, 
						 funcio_calculBlob *evaluador, 
						 int condition, 
						 double lowLimit, double highLimit /*=0*/)
							
{
	int i, numBlobs;
	bool resultavaluacio;
	double_stl_vector avaluacioBlobs;
	double_stl_vector::iterator itavaluacioBlobs;

	if( GetNumBlobs() <= 0 ) return;
	if( !evaluador ) return;
	//avaluem els blobs amb la funci�pertinent	
	avaluacioBlobs = GetSTLResult(evaluador);
	itavaluacioBlobs = avaluacioBlobs.begin();
	numBlobs = GetNumBlobs();
	switch(condition)
	{
		case B_EQUAL:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio= *itavaluacioBlobs == lowLimit;
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}				
			}
			break;
		case B_NOT_EQUAL:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio = *itavaluacioBlobs != lowLimit;
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
		case B_GREATER:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio= *itavaluacioBlobs > lowLimit;
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
		case B_LESS:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio= *itavaluacioBlobs < lowLimit;
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
		case B_GREATER_OR_EQUAL:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio= *itavaluacioBlobs>= lowLimit;
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
		case B_LESS_OR_EQUAL:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio= *itavaluacioBlobs <= lowLimit;
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
		case B_INSIDE:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio=( *itavaluacioBlobs >= lowLimit) && ( *itavaluacioBlobs <= highLimit); 
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
		case B_OUTSIDE:
			for(i=0;i<numBlobs;i++, itavaluacioBlobs++)
			{
				resultavaluacio=( *itavaluacioBlobs < lowLimit) || ( *itavaluacioBlobs > highLimit); 
				if( ( resultavaluacio && filterAction == B_INCLUDE ) ||
					( !resultavaluacio && filterAction == B_EXCLUDE ))
				{
					dst.m_blobs.push_back( new CBlob( GetBlob( i ) ));
				}
			}
			break;
	}


	// en cas de voler filtrar un CBlobResult i deixar-ho en el mateix CBlobResult
	// ( operacio inline )
	if( &dst == this ) 
	{
		// esborrem els primers blobs ( que s� els originals )
		// ja que els tindrem replicats al final si passen el filtre
		blob_vector::iterator itBlobs = m_blobs.begin();
		for( int i = 0; i < numBlobs; i++ )
		{
			delete *itBlobs;
			itBlobs++;
		}
		m_blobs.erase( m_blobs.begin(), itBlobs );
	}
}


/**
- FUNCI� GetBlob
- FUNCIONALITAT: Retorna un blob si aquest existeix (index != -1)
- PAR�ETRES:
	- indexblob: index del blob a retornar
- RESULTAT:
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/*
- FUNCTION: GetBlob
- FUNCTIONALITY: Gets the n-th blob (without ordering the blobs)
- PARAMETERS:
	- indexblob: index in the blob array
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
CBlob CBlobResult::GetBlob(int indexblob) const
{	
	if( indexblob < 0 || indexblob >= GetNumBlobs() )
		RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );

	return *m_blobs[indexblob];
}
CBlob *CBlobResult::GetBlob(int indexblob)
{	
	if( indexblob < 0 || indexblob >= GetNumBlobs() )
		RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );

	return m_blobs[indexblob];
}

/**
- FUNCI� GetNthBlob
- FUNCIONALITAT: Retorna l'en�sim blob segons un determinat criteri
- PAR�ETRES:
	- criteri: criteri per ordenar els blobs (objectes derivats de COperadorBlob)
	- nBlob: index del blob a retornar
	- dst: on es retorna el resultat
- RESULTAT:
	- retorna el blob nBlob a dst ordenant els blobs de la classe segons el criteri
	  en ordre DESCENDENT. Per exemple, per obtenir el blob major:
		GetNthBlob( CBlobGetArea(), 0, blobMajor );
		GetNthBlob( CBlobGetArea(), 1, blobMajor ); (segon blob m� gran)
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/*
- FUNCTION: GetNthBlob
- FUNCTIONALITY: Gets the n-th blob ordering first the blobs with some criteria
- PARAMETERS:
	- criteri: criteria to order the blob array
	- nBlob: index of the returned blob in the ordered blob array
	- dst: where to store the result
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
void CBlobResult::GetNthBlob( funcio_calculBlob *criteri, int nBlob, CBlob &dst ) const
{
	// verifiquem que no estem accedint fora el vector de blobs
	if( nBlob < 0 || nBlob >= GetNumBlobs() )
	{
		//RaiseError( EXCEPTION_BLOB_OUT_OF_BOUNDS );
		dst = CBlob();
		return;
	}

	double_stl_vector avaluacioBlobs, avaluacioBlobsOrdenat;
	double valorEnessim;

	//avaluem els blobs amb la funci�pertinent	
	avaluacioBlobs = GetSTLResult(criteri);

	avaluacioBlobsOrdenat = double_stl_vector( GetNumBlobs() );

	// obtenim els nBlob primers resultats (en ordre descendent)
	std::partial_sort_copy( avaluacioBlobs.begin(), 
						    avaluacioBlobs.end(),
						    avaluacioBlobsOrdenat.begin(), 
						    avaluacioBlobsOrdenat.end(),
						    std::greater<double>() );

	valorEnessim = avaluacioBlobsOrdenat[nBlob];

	// busquem el primer blob que t�el valor n-ssim
	double_stl_vector::const_iterator itAvaluacio = avaluacioBlobs.begin();

	bool trobatBlob = false;
	int indexBlob = 0;
	while( itAvaluacio != avaluacioBlobs.end() && !trobatBlob )
	{
		if( *itAvaluacio == valorEnessim )
		{
			trobatBlob = true;
			dst = CBlob( GetBlob(indexBlob));
		}
		itAvaluacio++;
		indexBlob++;
	}
}

/**
- FUNCI� ClearBlobs
- FUNCIONALITAT: Elimina tots els blobs de l'objecte
- PAR�ETRES:
- RESULTAT: 
	- Allibera tota la mem�ia dels blobs
- RESTRICCIONS:
- AUTOR: Ricard Borr� Navarra
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/*
- FUNCTION: ClearBlobs
- FUNCTIONALITY: Clears all the blobs from the object and releases all its memory
- PARAMETERS:
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
void CBlobResult::ClearBlobs()
{
	/*for( int i = 0; i < GetNumBlobs(); i++ )
	{
		delete m_blobs[i];
	}*/
	blob_vector::iterator itBlobs = m_blobs.begin();
	while( itBlobs != m_blobs.end() )
	{
		delete *itBlobs;
		itBlobs++;
	}

	m_blobs.clear();
}

/**
- FUNCI� RaiseError
- FUNCIONALITAT: Funci�per a notificar errors al l'usuari (en debug) i llen�
			   les excepcions
- PAR�ETRES:
	- errorCode: codi d'error
- RESULTAT: 
	- Ensenya un missatge a l'usuari (en debug) i llen� una excepci�
- RESTRICCIONS:
- AUTOR: Ricard Borr� Navarra
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/*
- FUNCTION: RaiseError
- FUNCTIONALITY: Error handling function
- PARAMETERS:
	- errorCode: reason of the error
- RESULT:
	- in _DEBUG version, shows a message box with the error. In release is silent.
	  In both cases throws an exception with the error.
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
void CBlobResult::RaiseError(const int errorCode) const
{
	// estem en mode debug?
#ifdef _DEBUG
	CString msg, format = "Error en CBlobResult: %s";

	switch (errorCode)
	{
	case EXCEPTION_BLOB_OUT_OF_BOUNDS:
		msg.Format(format, "Intentant accedir a un blob no existent");
		break;
	default:
		msg.Format(format, "Codi d'error desconegut");
		break;
	}

	AfxMessageBox(msg);

#endif
	throw errorCode;
}



/**************************************************************************
		Auxiliars / Auxiliary functions
**************************************************************************/


/**
- FUNCI� PrintBlobs
- FUNCIONALITAT: Escriu els par�etres (�ea, per�etre, exterior, mitjana) 
			   de tots els blobs a un fitxer.
- PAR�ETRES:
	- nom_fitxer: path complet del fitxer amb el resultat
- RESULTAT:
- RESTRICCIONS:
- AUTOR: Ricard Borr�
- DATA DE CREACI� 25-05-2005.
- MODIFICACI� Data. Autor. Descripci�
*/
/*
- FUNCTION: PrintBlobs
- FUNCTIONALITY: Prints some blob features in an ASCII file
- PARAMETERS:
	- nom_fitxer: full path + filename to generate
- RESULT:
- RESTRICTIONS:
- AUTHOR: Ricard Borr�
- CREATION DATE: 25-05-2005.
- MODIFICATION: Date. Author. Description.
*/
void CBlobResult::PrintBlobs( char *nom_fitxer ) const
{
	double_stl_vector area, /*perimetre,*/ exterior, mitjana, compacitat, longitud, 
					  externPerimeter, perimetreConvex, perimetre;
	int i;
	FILE *fitxer_sortida;

 	area      = GetSTLResult( CBlobGetArea());
	perimetre = GetSTLResult( CBlobGetPerimeter());
	exterior  = GetSTLResult( CBlobGetExterior());
	mitjana   = GetSTLResult( CBlobGetMean());
	compacitat = GetSTLResult(CBlobGetCompactness());
	longitud  = GetSTLResult( CBlobGetLength());
	externPerimeter = GetSTLResult( CBlobGetExternPerimeter());
	perimetreConvex = GetSTLResult( CBlobGetHullPerimeter());

	fitxer_sortida = fopen( nom_fitxer, "w" );

	for(i=0; i<GetNumBlobs(); i++)
	{
		fprintf( fitxer_sortida, "blob %d ->\t a=%7.0f\t p=%8.2f (%8.2f extern)\t pconvex=%8.2f\t ext=%.0f\t m=%7.2f\t c=%3.2f\t l=%8.2f\n",
				 i, area[i], perimetre[i], externPerimeter[i], perimetreConvex[i], exterior[i], mitjana[i], compacitat[i], longitud[i] );
	}
	fclose( fitxer_sortida );

}
