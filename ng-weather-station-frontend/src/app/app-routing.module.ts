import { NgModule }             from '@angular/core';
import { RouterModule, Routes } from '@angular/router';

import { AboutComponent } from './about/about.component';
import { DatapageComponent } from './datapage/datapage.component'
import { DatagraphComponent } from './datagraph/datagraph.component';

const routes: Routes = [
  { path: '', component: DatapageComponent },
  { path: 'about', component: AboutComponent },
  { path: 'data', component: DatapageComponent },
  { path: 'graph', component: DatagraphComponent}
];

@NgModule({
  imports: [ RouterModule.forRoot(routes) ],
  exports: [ RouterModule ]
})
export class AppRoutingModule {}