#include "compute_weights.h"
#include "merwe.h"

void compute_weights(MerweSigmaPoints *sp){


  double c = 0.5 / (sp->n + sp->lambda);
  for (int i=0; i < 2*sp->n + 1; i++) {
    sp->Wc[i] = c;
    sp->Wm[i] = c;
  } // end for loop

  sp->Wc[0] = sp->lambda / (sp->n + sp->lambda) +
    (1 - sp->alpha*sp->alpha + sp->beta);
  
  sp->Wm[0] = sp->lambda / (sp->n + sp->lambda);


} // end compute weights
