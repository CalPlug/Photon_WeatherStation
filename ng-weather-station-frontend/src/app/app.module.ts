import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';


import { AppComponent } from './app.component';
import { StdpageComponent } from './stdpage/stdpage.component';
import { AppRoutingModule } from './/app-routing.module';
import { AboutComponent } from './about/about.component';
import { DatapageComponent } from './datapage/datapage.component';


@NgModule({
  declarations: [
    AppComponent,
    StdpageComponent,
    AboutComponent,
    DatapageComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
