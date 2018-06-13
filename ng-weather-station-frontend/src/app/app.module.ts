import { DataService } from './data.service';
import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { HttpClientModule } from '@angular/common/http';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';


import { AppComponent } from './app.component';
import { StdpageComponent } from './stdpage/stdpage.component';
import { AppRoutingModule } from './/app-routing.module';
import { AboutComponent } from './about/about.component';
import { DatapageComponent } from './datapage/datapage.component';

import { NgDygraphsModule } from 'ng-dygraphs';
import { NewdtpComponent } from './newdtp/newdtp.component';
import { ChartsModule } from 'ng2-charts';
import { DatagraphComponent } from './datagraph/datagraph.component';

@NgModule({
  declarations: [
    AppComponent,
    StdpageComponent,
    AboutComponent,
    DatapageComponent,
    NewdtpComponent,
    DatagraphComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    HttpClientModule,
    BrowserAnimationsModule,
    NgDygraphsModule,
    FormsModule,
    ChartsModule
  ],
  providers: [DataService],
  bootstrap: [AppComponent]
})
export class AppModule { }
