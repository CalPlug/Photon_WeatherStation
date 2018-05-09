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

@NgModule({
  declarations: [
    AppComponent,
    StdpageComponent,
    AboutComponent,
    DatapageComponent,
    NewdtpComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    HttpClientModule,
    BrowserAnimationsModule,
    NgDygraphsModule,
    FormsModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
